package frc.robot.team3407;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * Acts like {@link edu.wpi.first.wpilibj.smartdashboard.SmartDashboard} but is not limited to the default NT parent table name,
 * and can be instanced so that different tables can be updated at different frequencies. Currently only useful for scheduling
 * {@link edu.wpi.first.util.sendable.Sendable} objects and not generic value pushes.
 */
public final class SenderNT {

	/**
	 * Represents a {@link Sendable} that may have sub-sendable members that should be setup with keys under the basename key.
	 */
	public static interface RecursiveSendable extends Sendable {

		/** Recursively initiate the sendables. */
		public void initRecursive(SenderNT inst, String base);

	}


	private NetworkTable table;
	private final Map<String, Sendable> data_tables = new HashMap<>();


	public SenderNT(String srcn) {
		this(NetworkTableInstance.getDefault().getTable(srcn));
	}
	public SenderNT(NetworkTable src) {
		this.table = src;
	}



	public synchronized void putData(String key, RecursiveSendable data) {
		data.initRecursive(this, key);
	}
	/* Copied straight from the SmartDashboard class. */
	public synchronized void putData(String key, Sendable data) {
		// try { this.putData( (RecursiveSendable)data ); } catch(ClassCastException e) {}	// this might create an infinite loop
		Sendable current = this.data_tables.get(key);
		if(current == null || current != data) {
			this.data_tables.put(key, data);

			SendableBuilderImpl builder = new SendableBuilderImpl();
			builder.setTable(this.table.getSubTable(key));
			SendableRegistry.publish(data, builder);
			builder.startListeners();

		}
	}
	public void putData(Sendable data) {
		String name = SendableRegistry.getName(data);
		if(!name.isEmpty()) {
			this.putData(name, data);
		}
	}

	public synchronized void updateValues() {
		for(Sendable data : this.data_tables.values()) {
			SendableRegistry.update(data);
		}
	}


}
