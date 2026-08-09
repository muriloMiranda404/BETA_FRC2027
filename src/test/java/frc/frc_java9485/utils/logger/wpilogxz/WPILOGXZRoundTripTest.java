package frc.frc_java9485.utils.logger.wpilogxz;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.tukaani.xz.XZInputStream;

import edu.wpi.first.util.datalog.DataLogRecord;


class WPILOGXZRoundTripTest {


    private static byte[] encode(EncoderBody body) throws IOException {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        WPILOGXZEncoder encoder = new WPILOGXZEncoder(out);
        encoder.writeHeader(WPILOGConstants.EXTRA_HEADER);
        body.write(encoder);
        encoder.close();
        return out.toByteArray();
    }


    private static WPILOGXZDecoder decode(byte[] compressed) throws IOException {
        try (XZInputStream in = new XZInputStream(new java.io.ByteArrayInputStream(compressed))) {
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            byte[] chunk = new byte[8192];
            int read;
            while ((read = in.read(chunk)) != -1) {
                out.write(chunk, 0, read);
            }
            return new WPILOGXZDecoder(ByteBuffer.wrap(out.toByteArray()));
        }
    }

    private interface EncoderBody {
        void write(WPILOGXZEncoder encoder) throws IOException;
    }

    private static List<DataLogRecord> recordsOf(WPILOGXZDecoder decoder) {
        List<DataLogRecord> records = new ArrayList<>();
        decoder.forEach(records::add);
        return records;
    }



    @Test
    void headerIsValidAndCarriesTheAdvantageKitMarker() throws IOException {
        WPILOGXZDecoder decoder = decode(encode(encoder -> {}));

        assertTrue(decoder.isValid(), "decoded log should have a valid WPILOG header");
        assertEquals(WPILOGConstants.EXTRA_HEADER, decoder.getExtraHeader());
        assertTrue(decoder.getVersion() >= 0x0100);
    }



    @Test
    void doubleRoundTrips() throws IOException {
        byte[] compressed = encode(encoder -> {
            int id = encoder.startEntry("/Test/Double", "double", WPILOGConstants.ENTRY_METADATA, 0);
            encoder.appendDouble(id, 1234.5678, 1000);
        });

        List<DataLogRecord> records = recordsOf(decode(compressed));
        assertEquals(2, records.size(), "expected one start record and one value record");
        assertTrue(records.get(0).isStart());
        assertEquals("/Test/Double", records.get(0).getStartData().name);
        assertEquals(1234.5678, records.get(1).getDouble(), 1e-9);
        assertEquals(1000, records.get(1).getTimestamp());
    }

    @Test
    void booleanIntegerFloatAndStringRoundTrip() throws IOException {
        byte[] compressed = encode(encoder -> {
            int boolId = encoder.startEntry("/Test/Bool", "boolean", WPILOGConstants.ENTRY_METADATA, 0);
            int intId = encoder.startEntry("/Test/Int", "int64", WPILOGConstants.ENTRY_METADATA, 0);
            int floatId = encoder.startEntry("/Test/Float", "float", WPILOGConstants.ENTRY_METADATA, 0);
            int stringId = encoder.startEntry("/Test/String", "string", WPILOGConstants.ENTRY_METADATA, 0);

            encoder.appendBoolean(boolId, true, 10);
            encoder.appendInteger(intId, 9_876_543_210L, 20);
            encoder.appendFloat(floatId, 3.5f, 30);
            encoder.appendString(stringId, "turret ready", 40);
        });

        List<DataLogRecord> records = recordsOf(decode(compressed));
        List<DataLogRecord> values = records.stream().filter(r -> !r.isControl()).toList();

        assertEquals(4, values.size());
        assertTrue(values.get(0).getBoolean());
        assertEquals(9_876_543_210L, values.get(1).getInteger());
        assertEquals(3.5f, values.get(2).getFloat(), 1e-6f);
        assertEquals("turret ready", values.get(3).getString());
    }


    @Test
    void negativeIntegerRoundTrips() throws IOException {
        byte[] compressed = encode(encoder -> {
            int id = encoder.startEntry("/Test/Neg", "int64", WPILOGConstants.ENTRY_METADATA, 0);
            encoder.appendInteger(id, -42L, 100);
        });

        List<DataLogRecord> values = recordsOf(decode(compressed)).stream()
                .filter(r -> !r.isControl())
                .toList();

        assertEquals(-42L, values.get(0).getInteger());
    }



    @Test
    void arraysRoundTrip() throws IOException {
        double[] doubles = {1.0, -2.5, 3.25};
        long[] longs = {1L, 2L, 300000L};
        boolean[] booleans = {true, false, true};
        String[] strings = {"a", "bb", "ccc"};

        byte[] compressed = encode(encoder -> {
            int doubleId = encoder.startEntry("/Test/DA", "double[]", WPILOGConstants.ENTRY_METADATA, 0);
            int longId = encoder.startEntry("/Test/IA", "int64[]", WPILOGConstants.ENTRY_METADATA, 0);
            int boolId = encoder.startEntry("/Test/BA", "boolean[]", WPILOGConstants.ENTRY_METADATA, 0);
            int stringId = encoder.startEntry("/Test/SA", "string[]", WPILOGConstants.ENTRY_METADATA, 0);

            encoder.appendDoubleArray(doubleId, doubles, 10);
            encoder.appendIntegerArray(longId, longs, 20);
            encoder.appendBooleanArray(boolId, booleans, 30);
            encoder.appendStringArray(stringId, strings, 40);
        });

        List<DataLogRecord> values = recordsOf(decode(compressed)).stream()
                .filter(r -> !r.isControl())
                .toList();

        assertArrayEquals(doubles, values.get(0).getDoubleArray(), 1e-9);
        assertArrayEquals(longs, values.get(1).getIntegerArray());
        assertArrayEquals(booleans, values.get(2).getBooleanArray());
        assertArrayEquals(strings, values.get(3).getStringArray());
    }

    @Test
    void rawRoundTrips() throws IOException {
        byte[] payload = {0x00, 0x7F, (byte) 0xFF, 0x10};

        byte[] compressed = encode(encoder -> {
            int id = encoder.startEntry("/Test/Raw", "raw", WPILOGConstants.ENTRY_METADATA, 0);
            encoder.appendRaw(id, payload, 50);
        });

        List<DataLogRecord> values = recordsOf(decode(compressed)).stream()
                .filter(r -> !r.isControl())
                .toList();

        assertArrayEquals(payload, values.get(0).getRaw());
    }



    @Test
    void entryIdsAreDistinctAndMetadataUpdatesRoundTrip() throws IOException {
        byte[] compressed = encode(encoder -> {
            int first = encoder.startEntry("/Test/A", "double", WPILOGConstants.ENTRY_METADATA, 0);
            int second = encoder.startEntry("/Test/B", "double", WPILOGConstants.ENTRY_METADATA, 0);
            assertEquals(1, first);
            assertEquals(2, second);
            encoder.setMetadata(first, "{\"source\":\"AdvantageKit\",\"unit\":\"m\"}", 5);
        });

        List<DataLogRecord> records = recordsOf(decode(compressed));
        DataLogRecord metadataRecord = records.stream().filter(DataLogRecord::isSetMetadata).findFirst().orElseThrow();

        assertEquals(1, metadataRecord.getSetMetadataData().entry);
        assertTrue(metadataRecord.getSetMetadataData().metadata.contains("\"unit\":\"m\""));
    }

    @Test
    void manySequentialValuesAllRoundTrip() throws IOException {
        int count = 500;

        byte[] compressed = encode(encoder -> {
            int id = encoder.startEntry("/Test/Ramp", "double", WPILOGConstants.ENTRY_METADATA, 0);
            for (int i = 0; i < count; i++) {
                encoder.appendDouble(id, i * 0.5, i * 20_000L);
            }
        });

        List<DataLogRecord> values = recordsOf(decode(compressed)).stream()
                .filter(r -> !r.isControl())
                .toList();

        assertEquals(count, values.size());
        for (int i = 0; i < count; i++) {
            assertEquals(i * 0.5, values.get(i).getDouble(), 1e-9);
            assertEquals(i * 20_000L, values.get(i).getTimestamp());
        }
    }


    @Test
    void repetitiveDataCompressesSubstantially() throws IOException {
        int count = 5000;

        byte[] compressed = encode(encoder -> {
            int id = encoder.startEntry("/Test/Flat", "double", WPILOGConstants.ENTRY_METADATA, 0);
            for (int i = 0; i < count; i++) {
                encoder.appendDouble(id, 3000.0, i * 20_000L);
            }
        });

        int uncompressedSize = decode(compressed).size();

        assertTrue(compressed.length < uncompressedSize / 4,
                "expected at least 4x compression, got " + uncompressedSize + " -> " + compressed.length);
    }
}
