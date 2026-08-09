// Adapted from FRC 6328 (Mechanical Advantage) — org.littletonrobotics.frc2026.util.logging.WPILOGXZDecoder.
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file
// at the root directory of the original project.

package frc.frc_java9485.utils.logger.wpilogxz;

import java.io.ByteArrayOutputStream;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.nio.BufferUnderflowException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.Iterator;
import java.util.NoSuchElementException;
import java.util.function.Consumer;

import org.tukaani.xz.XZInputStream;

import edu.wpi.first.util.datalog.DataLogRecord;


public class WPILOGXZDecoder implements Iterable<DataLogRecord> {

    private static final int HEADER_LENGTH = 12;
    private static final int DECOMPRESS_CHUNK = 8192;

    private static final Constructor<DataLogRecord> RECORD_CONSTRUCTOR;

    static {
        try {
            RECORD_CONSTRUCTOR =
                    DataLogRecord.class.getDeclaredConstructor(int.class, long.class, ByteBuffer.class);
            RECORD_CONSTRUCTOR.setAccessible(true);
        } catch (NoSuchMethodException e) {
            throw new ExceptionInInitializerError(
                    "WPILib changed the DataLogRecord constructor; WPILOGXZDecoder needs updating: " + e);
        }
    }

    private final ByteBuffer buffer;


    public WPILOGXZDecoder(ByteBuffer buffer) {
        this.buffer = buffer;
        this.buffer.order(ByteOrder.LITTLE_ENDIAN);
    }


    public WPILOGXZDecoder(String filename) throws IOException {
        try (FileInputStream fileStream = new FileInputStream(filename);
                XZInputStream xzStream = new XZInputStream(fileStream)) {
            this.buffer = ByteBuffer.wrap(decompress(xzStream)).order(ByteOrder.LITTLE_ENDIAN);
        }
    }


    private static byte[] decompress(InputStream stream) throws IOException {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        byte[] chunk = new byte[DECOMPRESS_CHUNK];
        try {
            int read;
            while ((read = stream.read(chunk)) != -1) {
                out.write(chunk, 0, read);
            }
        } catch (IOException e) {
            if (out.size() == 0) {
                throw e;
            }
        }
        return out.toByteArray();
    }


    public boolean isValid() {
        return buffer.remaining() >= HEADER_LENGTH
                && buffer.get(0) == 'W'
                && buffer.get(1) == 'P'
                && buffer.get(2) == 'I'
                && buffer.get(3) == 'L'
                && buffer.get(4) == 'O'
                && buffer.get(5) == 'G'
                && buffer.getShort(6) >= 0x0100;
    }


    public short getVersion() {
        return buffer.remaining() < HEADER_LENGTH ? 0 : buffer.getShort(6);
    }


    public String getExtraHeader() {
        ByteBuffer view = buffer.duplicate().order(ByteOrder.LITTLE_ENDIAN);
        view.position(8);
        byte[] extra = new byte[view.getInt()];
        view.get(extra);
        return new String(extra, StandardCharsets.UTF_8);
    }


    private int firstRecordPosition() {
        return HEADER_LENGTH + buffer.getInt(8);
    }

    @Override
    public void forEach(Consumer<? super DataLogRecord> action) {
        int size = buffer.remaining();
        for (int pos = firstRecordPosition(); pos < size; pos = getNextRecord(pos)) {
            DataLogRecord record;
            try {
                record = getRecord(pos);
            } catch (NoSuchElementException e) {
                break;
            }
            action.accept(record);
        }
    }

    @Override
    public Iterator<DataLogRecord> iterator() {
        return new Iterator<>() {
            private int pos = firstRecordPosition();

            @Override
            public boolean hasNext() {
                return pos < buffer.remaining();
            }

            @Override
            public DataLogRecord next() {
                DataLogRecord record = getRecord(pos);
                pos = getNextRecord(pos);
                return record;
            }
        };
    }


    public int size() {
        return buffer.remaining();
    }



    DataLogRecord getRecord(int pos) {
        try {
            int bitfield = buffer.get(pos) & 0xff;
            int entryLength = (bitfield & 0x3) + 1;
            int sizeLength = ((bitfield >> 2) & 0x3) + 1;
            int timestampLength = ((bitfield >> 4) & 0x7) + 1;
            int headerLength = 1 + entryLength + sizeLength + timestampLength;

            int entry = (int) readVarInt(pos + 1, entryLength);
            int size = (int) readVarInt(pos + 1 + entryLength, sizeLength);
            long timestamp = readVarInt(pos + 1 + entryLength + sizeLength, timestampLength);

            ByteBuffer data = buffer.duplicate();
            data.position(pos + headerLength);
            data.limit(pos + headerLength + size);

            try {
                return RECORD_CONSTRUCTOR.newInstance(entry, timestamp, data.slice());
            } catch (InstantiationException | IllegalAccessException | InvocationTargetException e) {
                throw new IllegalStateException("Failed to instantiate DataLogRecord", e);
            }
        } catch (BufferUnderflowException | IndexOutOfBoundsException e) {

            throw new NoSuchElementException();
        }
    }

    int getNextRecord(int pos) {
        int bitfield = buffer.get(pos) & 0xff;
        int entryLength = (bitfield & 0x3) + 1;
        int sizeLength = ((bitfield >> 2) & 0x3) + 1;
        int timestampLength = ((bitfield >> 4) & 0x7) + 1;
        int headerLength = 1 + entryLength + sizeLength + timestampLength;

        int size = 0;
        for (int i = 0; i < sizeLength; i++) {
            size |= (buffer.get(pos + 1 + entryLength + i) & 0xff) << (i * 8);
        }
        return pos + headerLength + size;
    }

    private long readVarInt(int pos, int length) {
        long value = 0;
        for (int i = 0; i < length; i++) {
            value |= ((long) (buffer.get(pos + i) & 0xff)) << (i * 8);
        }
        return value;
    }
}
