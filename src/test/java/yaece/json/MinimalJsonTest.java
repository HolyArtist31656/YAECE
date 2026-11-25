// Copyright (c) 2026 Yet Another Encounter Control Engine (YAECE) Contributors
// SPDX-License-Identifier: MIT

package yaece.json;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class MinimalJsonTest {

  @Test
  void parsesNestedObjectsArraysPrimitivesAndEscapes() {
    String json =
        """
        {
          "name": "YAECE\\n\\\"2027\\\"\\u0021",
          "intVal": 42,
          "negFloat": -3.14e-1,
          "flagTrue": true,
          "flagFalse": false,
          "empty": null,
          "arr": [1, 2.5, "x", []]
        }
        """;
    Map<String, Object> obj = MinimalJson.parseObject(json);
    assertEquals("YAECE\n\"2027\"!", MinimalJson.requireString(obj, "name", "root"));
    assertEquals(42, MinimalJson.requireInt(obj, "intVal", "root"));
    assertEquals(-0.314, MinimalJson.requireDouble(obj, "negFloat", "root"), 1e-9);
    assertTrue(MinimalJson.requireBoolean(obj, "flagTrue", "root"));
    assertFalse(MinimalJson.requireBoolean(obj, "flagFalse", "root"));
    assertNull(obj.get("empty"));
    List<Object> arr = MinimalJson.requireArray(obj, "arr", "root");
    assertEquals(4, arr.size());
    assertEquals(1L, arr.get(0));
    assertEquals(2.5, (Double) arr.get(1), 1e-9);
  }

  @Test
  void parsesFixedLengthDoubleArray() {
    Map<String, Object> obj = MinimalJson.parseObject("{\"dims\": [0.3, 0.12, 0.12]}");
    assertArrayEquals(
        new double[] {0.3, 0.12, 0.12},
        MinimalJson.requireDoubleArray(obj, "dims", 3, "piece"),
        1e-9);
  }

  @Test
  void rejectsMalformedJsonWithLineAndColumn() {
    JsonParseException ex =
        assertThrows(JsonParseException.class, () -> MinimalJson.parseObject("{\n  \"a\": 01\n}"));
    assertEquals(2, ex.getLine());
    assertTrue(ex.getColumn() > 0);
  }

  @Test
  void rejectsTrailingTokens() {
    assertThrows(JsonParseException.class, () -> MinimalJson.parseObject("{} {}"));
  }

  @Test
  void rejectsMissingRequiredField() {
    Map<String, Object> obj = MinimalJson.parseObject("{}");
    assertThrows(JsonParseException.class, () -> MinimalJson.requireInt(obj, "missing", "root"));
  }
}
