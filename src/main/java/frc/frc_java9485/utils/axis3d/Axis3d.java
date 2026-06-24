package frc.frc_java9485.utils.axis3d;

public enum Axis3d {
	X,
	Y,
	Z,
	ROLL,
	PITCH,
	YAW;

	public boolean isRotationAxis() {
		return switch (this) {
			case ROLL, PITCH, YAW -> true;
			case X, Y, Z -> false;
		};
	}

	public boolean isTranslationAxis() {
		return !isRotationAxis();
	}
}
