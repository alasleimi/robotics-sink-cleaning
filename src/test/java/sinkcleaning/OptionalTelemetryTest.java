package sinkcleaning;

import com.google.gson.*;
import java.io.*;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.Test;
import static org.junit.Assert.*;

public class OptionalTelemetryTest {
    @Test public void missingSensorDoesNotStopOtherChannelsOrRetryEverySample() throws Exception {
        StringWriter file = new StringWriter();
        OptionalTelemetry telemetry = new OptionalTelemetry(file);
        AtomicInteger calls = new AtomicInteger();
        for (int sample = 0; sample < 3; sample++) {
            assertNull(telemetry.read("missing", () -> {
                calls.incrementAndGet();
                throw new UnsupportedOperationException("Not available on this controller");
            }, value -> new JsonPrimitive(0)));
            assertEquals(4., telemetry.read("working", () -> 4., JsonPrimitive::new), 0.);
        }
        assertEquals(1, calls.get());
        assertEquals(3, telemetry.summary().getAsJsonObject("working").get("reads").getAsInt());
        JsonObject error = JsonParser.parseString(file.toString().split("\n")[0]).getAsJsonObject();
        assertTrue(error.get("value").isJsonNull());
        assertTrue(error.get("error").getAsString().contains("Not available"));
    }

    @Test public void missingSdkMethodNullAndInvalidNumbersAreOptional() throws Exception {
        OptionalTelemetry telemetry = new OptionalTelemetry(new StringWriter());
        assertNull(telemetry.<Double>read("old_sdk", () -> { throw new NoSuchMethodError("optional"); }, JsonPrimitive::new));
        assertNull(telemetry.read("null", () -> (Double) null, JsonPrimitive::new));
        assertNull(telemetry.read("nan", () -> new double[] {Double.NaN}, OptionalTelemetry::numbers));
        assertTrue(telemetry.summary().getAsJsonObject("old_sdk").get("disabled").getAsBoolean());
    }

    @Test public void slowChannelIsRecordedOnceThenDisabled() throws Exception {
        OptionalTelemetry telemetry = new OptionalTelemetry(new StringWriter());
        assertEquals(2., telemetry.read("slow", () -> { Thread.sleep(120); return 2.; }, JsonPrimitive::new), 0.);
        assertNull(telemetry.read("slow", () -> { fail("Retried slow sensor"); return 2.; }, JsonPrimitive::new));
    }

    @Test public void diskFailureIsNotMistakenForMissingSensor() throws Exception {
        Writer fullDisk = new Writer() {
            public void write(char[] data, int offset, int length) throws IOException { throw new IOException("disk full"); }
            public void flush() { }
            public void close() { }
        };
        try {
            new OptionalTelemetry(fullDisk).read("sensor", () -> 2., JsonPrimitive::new);
            fail("Disk error was hidden");
        } catch (IOException expected) {
            assertEquals("disk full", expected.getMessage());
        }
    }
}
