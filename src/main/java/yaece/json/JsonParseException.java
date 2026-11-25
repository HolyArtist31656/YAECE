// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.json;

/**
 * Unchecked exception thrown when {@link MinimalJson} encounters malformed JSON syntax or a missing
 * or mistyped field.
 */
public final class JsonParseException extends RuntimeException {
  private final int line;
  private final int column;

  /**
   * Creates a parse exception with source location diagnostics.
   *
   * @param message human-readable error description
   * @param line 1-based line number, or -1 if not tied to a parser position
   * @param column 1-based column number, or -1 if not tied to a parser position
   */
  public JsonParseException(String message, int line, int column) {
    super(line > 0 ? String.format("%s (line %d, column %d)", message, line, column) : message);
    this.line = line;
    this.column = column;
  }

  /** Creates a schema/accessor exception without a line/column location. */
  public JsonParseException(String message) {
    this(message, -1, -1);
  }

  /** Returns the 1-based line number where parsing failed, or -1 if unavailable. */
  public int getLine() {
    return line;
  }

  /** Returns the 1-based column number where parsing failed, or -1 if unavailable. */
  public int getColumn() {
    return column;
  }
}
