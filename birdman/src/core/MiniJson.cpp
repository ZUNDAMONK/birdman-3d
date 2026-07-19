#include "core/MiniJson.hpp"

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

namespace bm::json {

Value Value::null() { return {}; }
Value Value::booleanValue(bool v) { Value r; r.type = Type::Bool; r.boolean = v; return r; }
Value Value::numberValue(double v) { Value r; r.type = Type::Number; r.number = v; return r; }
Value Value::stringValue(std::string v) { Value r; r.type = Type::String; r.string = std::move(v); return r; }
Value Value::arrayValue(Array v) { Value r; r.type = Type::Array; r.array = std::move(v); return r; }
Value Value::objectValue(Object v) { Value r; r.type = Type::Object; r.object = std::move(v); return r; }

const Value* Value::find(const std::string& key) const {
    if (!isObject()) return nullptr;
    const auto it = object.find(key);
    return it == object.end() ? nullptr : &it->second;
}

namespace {

class Parser {
public:
    Parser(const std::string& text, ParseError* error) : text_(text), error_(error) {}

    bool run(Value& out) {
        skipWs();
        if (!value(out)) return false;
        skipWs();
        if (pos_ != text_.size()) return fail("trailing characters");
        return true;
    }

private:
    const std::string& text_;
    ParseError* error_ = nullptr;
    std::size_t pos_ = 0;
    bool failed_ = false;

    bool fail(const char* message) {
        if (!failed_ && error_) {
            error_->offset = pos_;
            error_->message = message;
        }
        failed_ = true;
        return false;
    }

    void skipWs() {
        while (pos_ < text_.size()) {
            const char c = text_[pos_];
            if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
            ++pos_;
        }
    }

    bool consume(char c) {
        if (pos_ >= text_.size() || text_[pos_] != c) return false;
        ++pos_;
        return true;
    }

    bool literal(const char* word) {
        const std::size_t start = pos_;
        for (const char* p = word; *p; ++p) {
            if (pos_ >= text_.size() || text_[pos_] != *p) {
                pos_ = start;
                return false;
            }
            ++pos_;
        }
        return true;
    }

    bool value(Value& out) {
        if (pos_ >= text_.size()) return fail("expected value");
        switch (text_[pos_]) {
        case 'n':
            if (!literal("null")) return fail("invalid literal");
            out = Value::null(); return true;
        case 't':
            if (!literal("true")) return fail("invalid literal");
            out = Value::booleanValue(true); return true;
        case 'f':
            if (!literal("false")) return fail("invalid literal");
            out = Value::booleanValue(false); return true;
        case '"': {
            std::string s;
            if (!string(s)) return false;
            out = Value::stringValue(std::move(s)); return true;
        }
        case '[': return array(out);
        case '{': return object(out);
        default: return number(out);
        }
    }

    static int hex(char c) {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
    }

    bool hex4(unsigned& cp) {
        if (pos_ + 4 > text_.size()) return fail("incomplete unicode escape");
        cp = 0;
        for (int i = 0; i < 4; ++i) {
            const int h = hex(text_[pos_++]);
            if (h < 0) return fail("invalid unicode escape");
            cp = (cp << 4) | (unsigned)h;
        }
        return true;
    }

    static void appendUtf8(std::string& out, unsigned cp) {
        if (cp <= 0x7f) out.push_back((char)cp);
        else if (cp <= 0x7ff) {
            out.push_back((char)(0xc0 | (cp >> 6)));
            out.push_back((char)(0x80 | (cp & 0x3f)));
        } else if (cp <= 0xffff) {
            out.push_back((char)(0xe0 | (cp >> 12)));
            out.push_back((char)(0x80 | ((cp >> 6) & 0x3f)));
            out.push_back((char)(0x80 | (cp & 0x3f)));
        } else {
            out.push_back((char)(0xf0 | (cp >> 18)));
            out.push_back((char)(0x80 | ((cp >> 12) & 0x3f)));
            out.push_back((char)(0x80 | ((cp >> 6) & 0x3f)));
            out.push_back((char)(0x80 | (cp & 0x3f)));
        }
    }

    bool string(std::string& out) {
        if (!consume('"')) return fail("expected string");
        while (pos_ < text_.size()) {
            const unsigned char c = (unsigned char)text_[pos_++];
            if (c == '"') return true;
            if (c < 0x20) return fail("control character in string");
            if (c != '\\') { out.push_back((char)c); continue; }
            if (pos_ >= text_.size()) return fail("incomplete escape");
            const char e = text_[pos_++];
            switch (e) {
            case '"': out.push_back('"'); break;
            case '\\': out.push_back('\\'); break;
            case '/': out.push_back('/'); break;
            case 'b': out.push_back('\b'); break;
            case 'f': out.push_back('\f'); break;
            case 'n': out.push_back('\n'); break;
            case 'r': out.push_back('\r'); break;
            case 't': out.push_back('\t'); break;
            case 'u': {
                unsigned cp = 0;
                if (!hex4(cp)) return false;
                if (cp >= 0xd800 && cp <= 0xdbff) {
                    if (pos_ + 2 > text_.size() || text_[pos_] != '\\' || text_[pos_ + 1] != 'u')
                        return fail("missing low surrogate");
                    pos_ += 2;
                    unsigned low = 0;
                    if (!hex4(low)) return false;
                    if (low < 0xdc00 || low > 0xdfff) return fail("invalid low surrogate");
                    cp = 0x10000 + ((cp - 0xd800) << 10) + (low - 0xdc00);
                } else if (cp >= 0xdc00 && cp <= 0xdfff) {
                    return fail("unexpected low surrogate");
                }
                appendUtf8(out, cp);
                break;
            }
            default: return fail("invalid escape");
            }
        }
        return fail("unterminated string");
    }

    bool number(Value& out) {
        const std::size_t start = pos_;
        if (consume('-') && pos_ >= text_.size()) return fail("invalid number");
        if (consume('0')) {
            if (pos_ < text_.size() && text_[pos_] >= '0' && text_[pos_] <= '9')
                return fail("leading zero in number");
        } else {
            if (pos_ >= text_.size() || text_[pos_] < '1' || text_[pos_] > '9')
                return fail("expected value");
            while (pos_ < text_.size() && text_[pos_] >= '0' && text_[pos_] <= '9') ++pos_;
        }
        if (consume('.')) {
            if (pos_ >= text_.size() || text_[pos_] < '0' || text_[pos_] > '9')
                return fail("invalid fraction");
            while (pos_ < text_.size() && text_[pos_] >= '0' && text_[pos_] <= '9') ++pos_;
        }
        if (pos_ < text_.size() && (text_[pos_] == 'e' || text_[pos_] == 'E')) {
            ++pos_;
            if (pos_ < text_.size() && (text_[pos_] == '+' || text_[pos_] == '-')) ++pos_;
            if (pos_ >= text_.size() || text_[pos_] < '0' || text_[pos_] > '9')
                return fail("invalid exponent");
            while (pos_ < text_.size() && text_[pos_] >= '0' && text_[pos_] <= '9') ++pos_;
        }
        const std::string token = text_.substr(start, pos_ - start);
        char* end = nullptr;
        const double n = std::strtod(token.c_str(), &end);
        if (!end || *end || !std::isfinite(n)) return fail("number out of range");
        out = Value::numberValue(n);
        return true;
    }

    bool array(Value& out) {
        consume('[');
        Value::Array a;
        skipWs();
        if (consume(']')) { out = Value::arrayValue(std::move(a)); return true; }
        while (true) {
            Value v;
            if (!value(v)) return false;
            a.push_back(std::move(v));
            skipWs();
            if (consume(']')) break;
            if (!consume(',')) return fail("expected ',' or ']'");
            skipWs();
        }
        out = Value::arrayValue(std::move(a));
        return true;
    }

    bool object(Value& out) {
        consume('{');
        Value::Object o;
        skipWs();
        if (consume('}')) { out = Value::objectValue(std::move(o)); return true; }
        while (true) {
            std::string key;
            if (!string(key)) return false;
            skipWs();
            if (!consume(':')) return fail("expected ':'");
            skipWs();
            Value v;
            if (!value(v)) return false;
            if (!o.emplace(std::move(key), std::move(v)).second)
                return fail("duplicate object key");
            skipWs();
            if (consume('}')) break;
            if (!consume(',')) return fail("expected ',' or '}'");
            skipWs();
        }
        out = Value::objectValue(std::move(o));
        return true;
    }
};

void writeEscaped(std::ostringstream& o, const std::string& s) {
    static const char* H = "0123456789abcdef";
    o << '"';
    for (unsigned char c : s) {
        switch (c) {
        case '"': o << "\\\""; break;
        case '\\': o << "\\\\"; break;
        case '\b': o << "\\b"; break;
        case '\f': o << "\\f"; break;
        case '\n': o << "\\n"; break;
        case '\r': o << "\\r"; break;
        case '\t': o << "\\t"; break;
        default:
            if (c < 0x20) o << "\\u00" << H[c >> 4] << H[c & 15];
            else o << (char)c;
        }
    }
    o << '"';
}

void writeValue(std::ostringstream& o, const Value& v) {
    switch (v.type) {
    case Value::Type::Null: o << "null"; break;
    case Value::Type::Bool: o << (v.boolean ? "true" : "false"); break;
    case Value::Type::Number:
        if (std::isfinite(v.number)) o << std::setprecision(17) << v.number;
        else o << "null";
        break;
    case Value::Type::String: writeEscaped(o, v.string); break;
    case Value::Type::Array:
        o << '[';
        for (std::size_t i = 0; i < v.array.size(); ++i) {
            if (i) o << ',';
            writeValue(o, v.array[i]);
        }
        o << ']';
        break;
    case Value::Type::Object: {
        o << '{';
        bool first = true;
        for (const auto& kv : v.object) {
            if (!first) o << ',';
            first = false;
            writeEscaped(o, kv.first);
            o << ':';
            writeValue(o, kv.second);
        }
        o << '}';
        break;
    }
    }
}

} // namespace

bool parse(const std::string& text, Value& out, ParseError* error) {
    if (error) *error = {};
    Value parsed;
    Parser p(text, error);
    if (!p.run(parsed)) return false;
    out = std::move(parsed);
    return true;
}

std::string stringify(const Value& value) {
    std::ostringstream o;
    writeValue(o, value);
    return o.str();
}

} // namespace bm::json
