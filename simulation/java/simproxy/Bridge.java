package simproxy;

import com.google.gson.*;
import java.net.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;

/** Only loopback HTTP is permitted. No controller address is read by this runtime. */
public final class Bridge {
  static final Gson JSON = new Gson();
  private final URI uri;
  private final String token;
  private JsonObject cachedState;
  private long cachedAt;

  public Bridge() throws Exception {
    JsonObject session =
        JsonParser.parseString(
                Files.readString(
                    Path.of(System.getProperty("sim.session", ".tools/adapter-session.json"))))
            .getAsJsonObject();
    uri = URI.create(session.get("url").getAsString());
    if (!"http".equals(uri.getScheme()) || !"127.0.0.1".equals(uri.getHost()))
      throw new IllegalArgumentException("Simulator bridge must use http://127.0.0.1");
    token = session.get("token").getAsString();
  }

  public static JsonObject object(Object... pairs) {
    JsonObject o = new JsonObject();
    for (int i = 0; i < pairs.length; i += 2)
      o.add((String) pairs[i], JSON.toJsonTree(pairs[i + 1]));
    return o;
  }

  public JsonObject call(String action, Object... pairs) {
    if (!action.equals("state") && !action.equals("motion") && !action.equals("trace")) cachedState=null;
    JsonObject request = object(pairs);
    request.addProperty("action", action);
    HttpURLConnection connection = null;
    try {
      connection = (HttpURLConnection) uri.toURL().openConnection();
      connection.setConnectTimeout(3000);
      connection.setReadTimeout(30000);
      connection.setRequestMethod("POST");
      connection.setDoOutput(true);
      connection.setRequestProperty("Authorization", "Bearer " + token);
      connection.setRequestProperty("Content-Type", "application/json");
      byte[] bytes = request.toString().getBytes(StandardCharsets.UTF_8);
      connection.setFixedLengthStreamingMode(bytes.length);
      try (java.io.OutputStream output = connection.getOutputStream()) {
        output.write(bytes);
      }
      int code = connection.getResponseCode();
      JsonObject result;
      try (java.io.InputStream input =
          code == 200 ? connection.getInputStream() : connection.getErrorStream()) {
        if (input == null) throw new IllegalStateException("Simulator HTTP status " + code);
        result =
            JsonParser.parseString(new String(input.readAllBytes(), StandardCharsets.UTF_8))
                .getAsJsonObject();
      }
      if (code != 200)
        throw new IllegalStateException("Simulator: " + result.get("error").getAsString());
      return result;
    } catch (java.io.IOException e) {
      if (connection != null) connection.disconnect();
      throw new IllegalStateException("Simulator bridge disconnected", e);
    }
  }

  public JsonObject state() {
    // Consecutive SDK telemetry getters share one 20 ms physics snapshot.
    if(cachedState==null || System.nanoTime()-cachedAt>=20_000_000L) {
      cachedState=call("state");
      cachedAt=System.nanoTime();
    }
    return cachedState;
  }

  static double[] array(JsonObject o, String key) {
    return JSON.fromJson(o.get(key), double[].class);
  }
}
