#pragma once

#include <cstddef>
#include <map>
#include <string>
#include <vector>

namespace bm::json {

struct Value {
    enum class Type { Null, Bool, Number, String, Array, Object };
    using Array = std::vector<Value>;
    using Object = std::map<std::string, Value>;

    Type type = Type::Null;
    bool boolean = false;
    double number = 0.0;
    std::string string;
    Array array;
    Object object;

    static Value null();
    static Value booleanValue(bool v);
    static Value numberValue(double v);
    static Value stringValue(std::string v);
    static Value arrayValue(Array v = {});
    static Value objectValue(Object v = {});

    bool isNull() const { return type == Type::Null; }
    bool isBool() const { return type == Type::Bool; }
    bool isNumber() const { return type == Type::Number; }
    bool isString() const { return type == Type::String; }
    bool isArray() const { return type == Type::Array; }
    bool isObject() const { return type == Type::Object; }

    const Value* find(const std::string& key) const;
};

struct ParseError {
    std::size_t offset = 0;
    std::string message;
};

// JSON全体を解析する。末尾の空白以外が残る場合も失敗する。
bool parse(const std::string& text, Value& out, ParseError* error = nullptr);

// std::mapのキー順を使う決定的なコンパクトJSON。
std::string stringify(const Value& value);

} // namespace bm::json
