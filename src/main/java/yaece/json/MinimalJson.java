// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.json;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Zero-dependency, strict RFC 8259 JSON parser and typed accessor utility for YAECE.
 *
 * <p>Designed to avoid third-party runtime JARs (such as Jackson or Gson) on the 2 GB SystemCore.
 */
public final class MinimalJson {
  private static final int MAX_DEPTH = 64;

  private final String src;
  private final int len;
  private int pos = 0;
  private int line = 1;
  private int col = 1;

  private MinimalJson(String src) {
    this.src = src;
    this.len = src.length();
  }

  /** Parses a JSON document and returns the root value. */
  public static Object parse(String json) {
    if (json == null) {
      throw new JsonParseException("JSON input must not be null");
    }
    MinimalJson parser = new MinimalJson(json);
    parser.skipWhitespace();
    Object value = parser.parseValue(0);
    parser.skipWhitespace();
    if (parser.pos < parser.len) {
      throw parser.error("Unexpected trailing character '" + parser.peek() + "'");
    }
    return value;
  }

  /** Parses a JSON document whose root must be a JSON object. */
  @SuppressWarnings("unchecked")
  public static Map<String, Object> parseObject(String json) {
    Object root = parse(json);
    if (!(root instanceof Map<?, ?> map)) {
      throw new JsonParseException("Expected root JSON object");
    }
    return (Map<String, Object>) map;
  }

  // -------------------------------------------------------------------------
  // Typed Accessor Helpers
  // -------------------------------------------------------------------------

  /** Returns a required nested JSON object. */
  @SuppressWarnings("unchecked")
  public static Map<String, Object> requireObject(
      Map<String, Object> obj, String key, String context) {
    Object val = requirePresent(obj, key, context);
    if (!(val instanceof Map<?, ?> map)) {
      throw new JsonParseException(context + "." + key + " must be a JSON object");
    }
    return (Map<String, Object>) map;
  }

  /** Returns an optional nested JSON object, or null if absent or null. */
  @SuppressWarnings("unchecked")
  public static Map<String, Object> optionalObject(
      Map<String, Object> obj, String key, String context) {
    Object val = obj.get(key);
    if (val == null) {
      return null;
    }
    if (!(val instanceof Map<?, ?> map)) {
      throw new JsonParseException(context + "." + key + " must be a JSON object");
    }
    return (Map<String, Object>) map;
  }

  /** Returns a required JSON array. */
  @SuppressWarnings("unchecked")
  public static List<Object> requireArray(Map<String, Object> obj, String key, String context) {
    Object val = requirePresent(obj, key, context);
    if (!(val instanceof List<?> list)) {
      throw new JsonParseException(context + "." + key + " must be a JSON array");
    }
    return (List<Object>) list;
  }

  /** Returns an optional JSON array, or an empty list if absent or null. */
  @SuppressWarnings("unchecked")
  public static List<Object> optionalArray(Map<String, Object> obj, String key, String context) {
    Object val = obj.get(key);
    if (val == null) {
      return List.of();
    }
    if (!(val instanceof List<?> list)) {
      throw new JsonParseException(context + "." + key + " must be a JSON array");
    }
    return (List<Object>) list;
  }

  /** Returns an optional list of non-blank strings. */
  public static List<String> optionalStringList(
      Map<String, Object> obj, String key, String context) {
    List<Object> raw = optionalArray(obj, key, context);
    List<String> result = new ArrayList<>(raw.size());
    for (int i = 0; i < raw.size(); i++) {
      Object elem = raw.get(i);
      if (!(elem instanceof String s) || s.isBlank()) {
        throw new JsonParseException(
            context + "." + key + "[" + i + "] must be a non-blank string");
      }
      result.add(s);
    }
    return result;
  }

  /** Returns a required non-blank JSON string. */
  public static String requireString(Map<String, Object> obj, String key, String context) {
    Object val = requirePresent(obj, key, context);
    if (!(val instanceof String str) || str.isBlank()) {
      throw new JsonParseException(context + "." + key + " must be a non-blank string");
    }
    return str;
  }

  /** Returns an optional JSON string, or {@code defaultValue} if absent or null. */
  public static String optionalString(
      Map<String, Object> obj, String key, String defaultValue, String context) {
    Object val = obj.get(key);
    if (val == null) {
      return defaultValue;
    }
    if (!(val instanceof String str)) {
      throw new JsonParseException(context + "." + key + " must be a string");
    }
    return str;
  }

  /** Returns a required finite double value. */
  public static double requireDouble(Map<String, Object> obj, String key, String context) {
    Object val = requirePresent(obj, key, context);
    return asFiniteDouble(val, context + "." + key);
  }

  /** Returns an optional finite double value, or {@code defaultValue} if absent or null. */
  public static double optionalDouble(
      Map<String, Object> obj, String key, double defaultValue, String context) {
    Object val = obj.get(key);
    if (val == null) {
      return defaultValue;
    }
    return asFiniteDouble(val, context + "." + key);
  }

  /** Returns a required exact integer value. */
  public static int requireInt(Map<String, Object> obj, String key, String context) {
    Object val = requirePresent(obj, key, context);
    if (val instanceof Long l) {
      if (l < Integer.MIN_VALUE || l > Integer.MAX_VALUE) {
        throw new JsonParseException(context + "." + key + " is out of 32-bit int range");
      }
      return l.intValue();
    }
    if (val instanceof Double d && Double.isFinite(d) && d == Math.rint(d)) {
      if (d < Integer.MIN_VALUE || d > Integer.MAX_VALUE) {
        throw new JsonParseException(context + "." + key + " is out of 32-bit int range");
      }
      return d.intValue();
    }
    throw new JsonParseException(context + "." + key + " must be an integer");
  }

  /** Returns an optional exact integer value, or {@code defaultValue} if absent or null. */
  public static int optionalInt(
      Map<String, Object> obj, String key, int defaultValue, String context) {
    Object val = obj.get(key);
    if (val == null) {
      return defaultValue;
    }
    return requireInt(obj, key, context);
  }

  /** Returns a required boolean value. */
  public static boolean requireBoolean(Map<String, Object> obj, String key, String context) {
    Object val = requirePresent(obj, key, context);
    if (!(val instanceof Boolean b)) {
      throw new JsonParseException(context + "." + key + " must be a boolean");
    }
    return b;
  }

  /** Returns an optional boolean value, or {@code defaultValue} if absent or null. */
  public static boolean optionalBoolean(
      Map<String, Object> obj, String key, boolean defaultValue, String context) {
    Object val = obj.get(key);
    if (val == null) {
      return defaultValue;
    }
    if (!(val instanceof Boolean b)) {
      throw new JsonParseException(context + "." + key + " must be a boolean");
    }
    return b;
  }

  /** Converts a parsed JSON array element into a finite double. */
  public static double asFiniteDouble(Object val, String path) {
    if (val instanceof Number n) {
      double d = n.doubleValue();
      if (!Double.isFinite(d)) {
        throw new JsonParseException(path + " must be a finite number");
      }
      return d;
    }
    throw new JsonParseException(path + " must be a number");
  }

  /** Converts a parsed JSON array of numbers into a fixed-length {@code double[]}. */
  public static double[] requireDoubleArray(
      Map<String, Object> obj, String key, int expectedLength, String context) {
    List<Object> list = requireArray(obj, key, context);
    if (list.size() != expectedLength) {
      throw new JsonParseException(
          String.format(
              "%s.%s must contain exactly %d numbers (found %d)",
              context, key, expectedLength, list.size()));
    }
    double[] result = new double[expectedLength];
    for (int i = 0; i < expectedLength; i++) {
      result[i] = asFiniteDouble(list.get(i), context + "." + key + "[" + i + "]");
    }
    return result;
  }

  /** Casts a JSON array element to a JSON object map. */
  @SuppressWarnings("unchecked")
  public static Map<String, Object> asObject(Object val, String path) {
    if (!(val instanceof Map<?, ?> map)) {
      throw new JsonParseException(path + " must be a JSON object");
    }
    return (Map<String, Object>) map;
  }

  private static Object requirePresent(Map<String, Object> obj, String key, String context) {
    if (!obj.containsKey(key) || obj.get(key) == null) {
      throw new JsonParseException("Missing required field: " + context + "." + key);
    }
    return obj.get(key);
  }

  // -------------------------------------------------------------------------
  // Recursive-Descent Lexer / Parser
  // -------------------------------------------------------------------------

  private Object parseValue(int depth) {
    if (depth > MAX_DEPTH) {
      throw error("Maximum JSON nesting depth (" + MAX_DEPTH + ") exceeded");
    }
    if (pos >= len) {
      throw error("Unexpected end of JSON input");
    }
    char c = peek();
    return switch (c) {
      case '{' -> parseObjectInternal(depth + 1);
      case '[' -> parseArrayInternal(depth + 1);
      case '"' -> parseString();
      case 't' -> parseLiteral("true", Boolean.TRUE);
      case 'f' -> parseLiteral("false", Boolean.FALSE);
      case 'n' -> parseLiteral("null", null);
      default -> {
        if (c == '-' || (c >= '0' && c <= '9')) {
          yield parseNumber();
        }
        throw error("Unexpected character '" + c + "'");
      }
    };
  }

  private Map<String, Object> parseObjectInternal(int depth) {
    expect('{');
    skipWhitespace();
    Map<String, Object> map = new LinkedHashMap<>();
    if (pos < len && peek() == '}') {
      advance();
      return Collections.unmodifiableMap(map);
    }
    while (true) {
      skipWhitespace();
      if (pos >= len || peek() != '"') {
        throw error("Expected string key in JSON object");
      }
      String key = parseString();
      skipWhitespace();
      expect(':');
      skipWhitespace();
      Object val = parseValue(depth);
      map.put(key, val);
      skipWhitespace();
      if (pos >= len) {
        throw error("Unterminated JSON object");
      }
      char next = peek();
      if (next == '}') {
        advance();
        return Collections.unmodifiableMap(map);
      }
      if (next != ',') {
        throw error("Expected ',' or '}' in JSON object, found '" + next + "'");
      }
      advance();
    }
  }

  private List<Object> parseArrayInternal(int depth) {
    expect('[');
    skipWhitespace();
    List<Object> list = new ArrayList<>();
    if (pos < len && peek() == ']') {
      advance();
      return Collections.unmodifiableList(list);
    }
    while (true) {
      skipWhitespace();
      list.add(parseValue(depth));
      skipWhitespace();
      if (pos >= len) {
        throw error("Unterminated JSON array");
      }
      char next = peek();
      if (next == ']') {
        advance();
        return Collections.unmodifiableList(list);
      }
      if (next != ',') {
        throw error("Expected ',' or ']' in JSON array, found '" + next + "'");
      }
      advance();
    }
  }

  private String parseString() {
    expect('"');
    StringBuilder sb = new StringBuilder();
    while (pos < len) {
      char c = advance();
      if (c == '"') {
        return sb.toString();
      }
      if (c < 0x20) {
        throw error("Unescaped control character in JSON string");
      }
      if (c == '\\') {
        if (pos >= len) {
          throw error("Unterminated escape sequence in JSON string");
        }
        char esc = advance();
        switch (esc) {
          case '"' -> sb.append('"');
          case '\\' -> sb.append('\\');
          case '/' -> sb.append('/');
          case 'b' -> sb.append('\b');
          case 'f' -> sb.append('\f');
          case 'n' -> sb.append('\n');
          case 'r' -> sb.append('\r');
          case 't' -> sb.append('\t');
          case 'u' -> sb.append(parseHex4());
          default -> throw error("Invalid escape sequence '\\" + esc + "'");
        }
      } else {
        sb.append(c);
      }
    }
    throw error("Unterminated JSON string");
  }

  private char parseHex4() {
    if (pos + 4 > len) {
      throw error("Incomplete \\uXXXX unicode escape");
    }
    int code = 0;
    for (int i = 0; i < 4; i++) {
      char h = advance();
      int digit = Character.digit(h, 16);
      if (digit < 0) {
        throw error("Invalid hex digit '" + h + "' in unicode escape");
      }
      code = (code << 4) | digit;
    }
    return (char) code;
  }

  private Number parseNumber() {
    int start = pos;
    boolean isFloat = false;
    if (peek() == '-') {
      advance();
    }
    if (pos >= len) {
      throw error("Unexpected end of number");
    }
    if (peek() == '0') {
      advance();
      if (pos < len && peek() >= '0' && peek() <= '9') {
        throw error("Leading zeros are not permitted in JSON numbers");
      }
    } else if (peek() >= '1' && peek() <= '9') {
      while (pos < len && peek() >= '0' && peek() <= '9') {
        advance();
      }
    } else {
      throw error("Invalid number syntax");
    }

    if (pos < len && peek() == '.') {
      isFloat = true;
      advance();
      if (pos >= len || peek() < '0' || peek() > '9') {
        throw error("Expected digit after decimal point");
      }
      while (pos < len && peek() >= '0' && peek() <= '9') {
        advance();
      }
    }

    if (pos < len && (peek() == 'e' || peek() == 'E')) {
      isFloat = true;
      advance();
      if (pos < len && (peek() == '+' || peek() == '-')) {
        advance();
      }
      if (pos >= len || peek() < '0' || peek() > '9') {
        throw error("Expected digit in exponent");
      }
      while (pos < len && peek() >= '0' && peek() <= '9') {
        advance();
      }
    }

    String token = src.substring(start, pos);
    try {
      if (!isFloat) {
        return Long.parseLong(token);
      }
      double d = Double.parseDouble(token);
      if (!Double.isFinite(d)) {
        throw error("Numeric value overflows finite IEEE-754 double: " + token);
      }
      return d;
    } catch (NumberFormatException ex) {
      if (!isFloat) {
        double d = Double.parseDouble(token);
        if (Double.isFinite(d)) {
          return d;
        }
      }
      throw error("Invalid number literal: " + token);
    }
  }

  private Object parseLiteral(String literal, Object value) {
    if (pos + literal.length() > len || !src.startsWith(literal, pos)) {
      throw error("Expected literal '" + literal + "'");
    }
    for (int i = 0; i < literal.length(); i++) {
      advance();
    }
    return value;
  }

  private void skipWhitespace() {
    while (pos < len) {
      char c = peek();
      if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
        advance();
      } else {
        break;
      }
    }
  }

  private void expect(char expected) {
    if (pos >= len || peek() != expected) {
      throw error(
          "Expected '"
              + expected
              + "'"
              + (pos < len ? ", found '" + peek() + "'" : ", found end of input"));
    }
    advance();
  }

  private char peek() {
    return src.charAt(pos);
  }

  private char advance() {
    char c = src.charAt(pos++);
    if (c == '\n') {
      line++;
      col = 1;
    } else {
      col++;
    }
    return c;
  }

  private JsonParseException error(String message) {
    return new JsonParseException(message, line, col);
  }
}
