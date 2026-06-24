package frc.frc_java9485.utils.axis3d;

@SuppressWarnings("rawtypes")
public enum TranslationAxis3d implements Axis3dConvertable {
	X,
	Y,
	Z;

	public Axis3d toAxis3d() {
		return switch (this) {
			case X -> Axis3d.X;
			case Y -> Axis3d.Y;
			case Z -> Axis3d.Z;
		};
	}
}
