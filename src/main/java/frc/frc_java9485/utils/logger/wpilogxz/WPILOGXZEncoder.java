// Adapted from FRC 6328 (Mechanical Advantage) — org.littletonrobotics.frc2026.util.logging.WPILOGXZEncoder.
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file
// at the root directory of the original project.

package frc.frc_java9485.utils.logger.wpilogxz;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;

import org.tukaani.xz.LZMA2Options;
import org.tukaani.xz.XZOutputStream;


public class WPILOGXZEncoder {


    private static final int COMPRESSION_PRESET = 6;


    private static final int MAX_HEADER_BYTES = 17;

    private final XZOutputStream outputStream;
    private int nextEntryId = 1;


    private final ByteArrayOutputStream payloadBuffer = new ByteArrayOutputStream(1024);
    private final ByteBuffer primitiveBuffer = ByteBuffer.allocate(8).order(ByteOrder.LITTLE_ENDIAN);
    private final byte[] headerBuffer = new byte[MAX_HEADER_BYTES];

    public WPILOGXZEncoder(OutputStream out) throws IOException {
        this.outputStream = new XZOutputStream(out, new LZMA2Options(COMPRESSION_PRESET));
    }

    public void close() throws IOException {
        outputStream.close();
    }

    public void flush() throws IOException {
        outputStream.flush();
    }


    public void writeHeader(String extraHeader) throws IOException {
        outputStream.write("WPILOG".getBytes(StandardCharsets.UTF_8));
        outputStream.write(0x00);
        outputStream.write(0x01);

        byte[] extraBytes = extraHeader.getBytes(StandardCharsets.UTF_8);
        ByteBuffer lengthBuffer = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN);
        lengthBuffer.putInt(extraBytes.length);
        outputStream.write(lengthBuffer.array());
        outputStream.write(extraBytes);
    }


    public int startEntry(String name, String type, String metadata, long timestamp) throws IOException {
        int id = nextEntryId++;
        payloadBuffer.reset();
        payloadBuffer.write(0);
        writeIntToPayload(id, 4);
        writeStringToPayload(name);
        writeStringToPayload(type);
        writeStringToPayload(metadata);
        writeRecord(0, timestamp, payloadBuffer.toByteArray());
        return id;
    }


    public void setMetadata(int id, String metadata, long timestamp) throws IOException {
        payloadBuffer.reset();
        payloadBuffer.write(2);
        writeIntToPayload(id, 4);
        writeStringToPayload(metadata);
        writeRecord(0, timestamp, payloadBuffer.toByteArray());
    }



    public void appendRaw(int id, byte[] value, long timestamp) throws IOException {
        writeRecord(id, timestamp, value);
    }

    public void appendBoolean(int id, boolean value, long timestamp) throws IOException {
        writeRecord(id, timestamp, new byte[] {(byte) (value ? 1 : 0)});
    }

    public void appendInteger(int id, long value, long timestamp) throws IOException {
        primitiveBuffer.clear();
        primitiveBuffer.putLong(value);
        writeRecord(id, timestamp, primitiveBuffer.array());
    }

    public void appendFloat(int id, float value, long timestamp) throws IOException {
        primitiveBuffer.clear();
        primitiveBuffer.putFloat(value);
        writeRecord(id, timestamp, primitiveBuffer.array(), 4);
    }

    public void appendDouble(int id, double value, long timestamp) throws IOException {
        primitiveBuffer.clear();
        primitiveBuffer.putDouble(value);
        writeRecord(id, timestamp, primitiveBuffer.array());
    }

    public void appendString(int id, String value, long timestamp) throws IOException {
        writeRecord(id, timestamp, value.getBytes(StandardCharsets.UTF_8));
    }

    public void appendBooleanArray(int id, boolean[] value, long timestamp) throws IOException {
        byte[] bytes = new byte[value.length];
        for (int i = 0; i < value.length; i++) {
            bytes[i] = (byte) (value[i] ? 1 : 0);
        }
        writeRecord(id, timestamp, bytes);
    }

    public void appendIntegerArray(int id, long[] value, long timestamp) throws IOException {
        ByteBuffer buffer = ByteBuffer.allocate(value.length * 8).order(ByteOrder.LITTLE_ENDIAN);
        for (long v : value) {
            buffer.putLong(v);
        }
        writeRecord(id, timestamp, buffer.array());
    }

    public void appendFloatArray(int id, float[] value, long timestamp) throws IOException {
        ByteBuffer buffer = ByteBuffer.allocate(value.length * 4).order(ByteOrder.LITTLE_ENDIAN);
        for (float v : value) {
            buffer.putFloat(v);
        }
        writeRecord(id, timestamp, buffer.array());
    }

    public void appendDoubleArray(int id, double[] value, long timestamp) throws IOException {
        ByteBuffer buffer = ByteBuffer.allocate(value.length * 8).order(ByteOrder.LITTLE_ENDIAN);
        for (double v : value) {
            buffer.putDouble(v);
        }
        writeRecord(id, timestamp, buffer.array());
    }

    public void appendStringArray(int id, String[] value, long timestamp) throws IOException {
        ByteArrayOutputStream bytes = new ByteArrayOutputStream();
        ByteBuffer lengthBuffer = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN);

        lengthBuffer.putInt(value.length);
        bytes.write(lengthBuffer.array());

        for (String s : value) {
            byte[] stringBytes = s.getBytes(StandardCharsets.UTF_8);
            lengthBuffer.clear();
            lengthBuffer.putInt(stringBytes.length);
            bytes.write(lengthBuffer.array());
            bytes.write(stringBytes);
        }
        writeRecord(id, timestamp, bytes.toByteArray());
    }



    private void writeRecord(int entryId, long timestamp, byte[] payload) throws IOException {
        writeRecord(entryId, timestamp, payload, payload.length);
    }


    private void writeRecord(int entryId, long timestamp, byte[] payload, int length) throws IOException {
        int idLength = bytesNeeded(entryId);
        int sizeLength = bytesNeeded(length);
        int timestampLength = bytesNeeded(timestamp);

        int bitfield = (idLength - 1) | ((sizeLength - 1) << 2) | ((timestampLength - 1) << 4);

        int pos = 0;
        headerBuffer[pos++] = (byte) bitfield;
        pos = writeVarInt(entryId, idLength, pos);
        pos = writeVarInt(length, sizeLength, pos);
        pos = writeVarInt(timestamp, timestampLength, pos);

        outputStream.write(headerBuffer, 0, pos);
        outputStream.write(payload, 0, length);
    }


    private int writeVarInt(long value, int length, int pos) {
        long remaining = value;
        for (int i = 0; i < length; i++) {
            headerBuffer[pos++] = (byte) (remaining & 0xFF);
            remaining >>>= 8;
        }
        return pos;
    }


    private static int bytesNeeded(long value) {
        if (value == 0) {
            return 1;
        }
        if (value < 0) {
            return 8;
        }
        if (value < (1L << 8)) {
            return 1;
        }
        if (value < (1L << 16)) {
            return 2;
        }
        if (value < (1L << 24)) {
            return 3;
        }
        if (value < (1L << 32)) {
            return 4;
        }
        if (value < (1L << 40)) {
            return 5;
        }
        if (value < (1L << 48)) {
            return 6;
        }
        if (value < (1L << 56)) {
            return 7;
        }
        return 8;
    }

    private void writeIntToPayload(int value, int length) {
        for (int i = 0; i < length; i++) {
            payloadBuffer.write((value >> (i * 8)) & 0xFF);
        }
    }

    private void writeStringToPayload(String value) throws IOException {
        byte[] bytes = value.getBytes(StandardCharsets.UTF_8);
        writeIntToPayload(bytes.length, 4);
        payloadBuffer.write(bytes);
    }
}
