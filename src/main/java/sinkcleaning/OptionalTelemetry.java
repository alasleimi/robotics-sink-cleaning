package sinkcleaning;

import com.google.gson.*;
import java.io.IOException;
import java.io.Writer;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.function.Function;

/** Best-effort SDK reads. File-writing errors are deliberately NOT swallowed. */
final class OptionalTelemetry {
    private final Writer output;
    private final Map<String, JsonObject> channels = new LinkedHashMap<>();
    private long started;
    private int sample = -1;

    OptionalTelemetry(Writer output) {
        this.output = output;
    }

    void start(long started) {
        this.started = started;
    }

    void sample(int sample) {
        this.sample = sample;
    }

    <T> T read(String name, Callable<T> getter, Function<T, JsonElement> encode) throws IOException {
        JsonObject status = channels.computeIfAbsent(name, key -> {
            JsonObject value = new JsonObject();
            value.addProperty("reads", 0);
            value.addProperty("disabled", false);
            return value;
        });
        if (status.get("disabled").getAsBoolean()) return null;

        long began = System.nanoTime();
        JsonObject event = new JsonObject();
        event.addProperty("channel", name);
        event.addProperty("sample", sample);
        event.add("t", sample < 0 ? JsonNull.INSTANCE : new JsonPrimitive((began - started) * 1e-9));
        T value = null;
        try {
            value = getter.call();
            if (value == null) throw new IllegalStateException("SDK returned no value");
            JsonElement encoded = encode.apply(value);
            if (encoded == null || encoded.isJsonNull()) throw new IllegalStateException("No usable value");
            event.add("value", encoded);
            status.addProperty("reads", status.get("reads").getAsInt() + 1);
        } catch (Exception | LinkageError error) {
            // A missing API or unavailable sensor must not discard the tool path.
            value = null;
            event.add("value", JsonNull.INSTANCE);
            disable(name, status, error.getClass().getSimpleName() + ": " + error.getMessage());
            event.addProperty("error", status.get("reason").getAsString());
        }
        double elapsed = (System.nanoTime() - began) * 1e-9;
        event.addProperty("read_seconds", elapsed);
        status.addProperty("last_read_seconds", elapsed);
        if (elapsed > .1 && !status.get("disabled").getAsBoolean()) {
            disable(name, status, "Read exceeded 100 ms; disabled to protect path sampling");
            event.addProperty("warning", status.get("reason").getAsString());
        }
        output.write(event.toString());
        output.write('\n');
        return value;
    }

    private void disable(String name, JsonObject status, String reason) {
        status.addProperty("disabled", true);
        status.addProperty("reason", reason);
        System.err.println("Optional telemetry unavailable: " + name + " (" + reason + "). Continuing recording.");
    }

    JsonObject summary() {
        JsonObject result = new JsonObject();
        channels.forEach(result::add);
        return result;
    }

    static JsonArray numbers(double... values) {
        JsonArray result = new JsonArray();
        for (double value : values) {
            if (!Double.isFinite(value)) throw new IllegalArgumentException("Nonfinite sensor value");
            result.add(value);
        }
        return result;
    }
}
