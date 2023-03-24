package frc.robot.team3407.controls;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.team3407.controls.Input.*;


/** ControlSchemeManager manages an array of control schemes and automatically assigns input-action bindings depending on the inputs that are available */
public class ControlSchemeManager {

	/** ControlSchemeBase defines the requirements for a compatible control scheme */
	public static interface ControlSchemeBase {
		public static interface Compat_F {
			public InputDevice[] test(InputDevice... inputs);
		}
		public static interface Setup_F {
			public void run(InputDevice... inputs);
		}

		/** Determines if the inputs currently connected to the DS are sufficient to initialize the control scheme
		 * @param inputs an array of available inputs - usually of length DriverStation.kJoystickPorts
		 * @return an array of inputs that should be passed to setup() to initialize the control scheme, or null if the requirements are not met
		 */
		public InputDevice[] compatible(InputDevice... inputs);
		/** Initializes the control scheme.
		 * @param inputs The input array as returned directly from compatible(). The order of inputs in that array is the same as in this array
		 */
		public void setup(InputDevice... inputs);
		/** Optional callback for deiniting the scheduled bindings */
		default public void shutdown() {}
		/** What name should the selector use to identify this control scheme
		 * @return The name or description of the control scheme
		 */
		public String getDesc();
	}
	public static class ControlScheme implements ControlSchemeBase {
		private final Compat_F compatibility;
		private final Setup_F setup;
		private final Runnable shutdown;
		private final String desc;
		
		public ControlScheme(String d, Compat_F c, Setup_F s) { this(d, c, s, null); }
		public ControlScheme(String d, Compat_F c, Setup_F s, Runnable e) {
			this.compatibility = c;
			this.setup = s;
			this.shutdown = e;
			this.desc = d;
		}

		@Override
		public String getDesc() {
			return this.desc;
		}
		@Override
		public InputDevice[] compatible(InputDevice... inputs) {
			return this.compatibility.test(inputs);
		}
		@Override
		public void setup(InputDevice... inputs) {
			this.setup.run(inputs);
		}
		@Override
		public void shutdown() {
			if(this.shutdown != null) {
				this.shutdown.run();
			}
		}

	}
	public static class AutomatedTester implements ControlSchemeBase.Compat_F {
		private final InputMap[] requirements;
		private final InputDevice[] buff;	// the array [reference] is final but the assignments are not

		public AutomatedTester(InputMap... reqs) {
			this.requirements = reqs;
			this.buff = new InputDevice[reqs.length];
		}

		@Override	/* This is basically just a big matching function */
		public InputDevice[] test(InputDevice... inputs) {
			int[] avail, reqs = new int[this.requirements.length];	// look up tables for inputs (param) and this.requirements, respectively
			int found = 0;
			if(inputs.length == DriverStation.kJoystickPorts) {
				avail = new int[]{0, 1, 2, 3, 4, 5};	// 0 through (DriverStation.kJoystickPorts - 1)
			} else {
				avail = new int[inputs.length];
				for(int i = 0; i < avail.length; i++) {
					avail[i] = inputs[i].getPort();
				}
			}
			for(int i = 0; i < reqs.length; i++) {
				reqs[i] = i;
			}
			for(int i = found; i < avail.length; i++) {		// for each remaining port:
				int p = avail[i];
				for(int r = found; r < reqs.length; r++) {			// for each remaining requirement:
					if(this.requirements[reqs[r]].compatible(p)) {			// if compatible:
						this.buff[reqs[r]] = inputs[p];
						if(i != found) {
							for(int q = i; q > found; q--) {	// start at i, work backwards until @ found
								avail[q] = avail[q - 1];		// bump all vals before i up an index
							}
						}
						if(r != found) {
							for(int q = r; q > found; q--) {	// same as above but for the reqs look up table
								reqs[q] = reqs[q - 1];
							}
						}
						found++;
						break;
					}
				}
				if(found == this.requirements.length) {
					return this.buff;
				}
			}
			return null;
		}

	}



	public static enum AmbiguousSolution {
		NONE,
		PREFER_COMPLEX,
		PREFER_SIMPLE
	}

	private final SendableChooser<Integer> options = new SendableChooser<>();
	private final ArrayList<ControlSchemeBase> schemes = new ArrayList<>();
	private final InputDevice[] inputs = new InputDevice[DriverStation.kJoystickPorts];
	private Thread searcher;
	private AmbiguousSolution amb_preference = AmbiguousSolution.NONE;

	public ControlSchemeManager() {
		this.options.setDefaultOption("Automatic", Integer.valueOf(-1));
		for(int i = 0; i < this.inputs.length; i++) {
			this.inputs[i] = new InputDevice(i);
		}
	}

	public void addScheme(ControlSchemeBase c) {
		this.options.addOption(c.getDesc(), Integer.valueOf(this.schemes.size()));
		this.schemes.add(c);
	}
	public void addScheme(String d, ControlSchemeBase.Compat_F c, ControlSchemeBase.Setup_F s) {
		this.options.addOption(d, Integer.valueOf(this.schemes.size()));
		this.schemes.add(new ControlScheme(d, c, s));
	}
	public void addScheme(String d, ControlSchemeBase.Compat_F c, ControlSchemeBase.Setup_F s, Runnable e) {
		this.options.addOption(d, Integer.valueOf(this.schemes.size()));
		this.schemes.add(new ControlScheme(d, c, s, e));
	}
	public void setDefault(ControlSchemeBase c) {
		this.options.setDefaultOption(c.getDesc(), Integer.valueOf(this.schemes.size()));
		this.schemes.add(c);
	}
	public void setDefault(String d, ControlSchemeBase.Compat_F c, ControlSchemeBase.Setup_F s) {
		this.options.setDefaultOption(d, Integer.valueOf(this.schemes.size()));
		this.schemes.add(new ControlScheme(d, c, s));
	}
	public void setDefault(String d, ControlSchemeBase.Compat_F c, ControlSchemeBase.Setup_F s, Runnable e) {
		this.options.setDefaultOption(d, Integer.valueOf(this.schemes.size()));
		this.schemes.add(new ControlScheme(d, c, s, e));
	}
	public void publishSelector() { this.publishSelector("Control Scheme"); }
	public void publishSelector(String n) {
		SmartDashboard.putData(n, this.options);
	}
	public void setAmbiguousSolution(AmbiguousSolution s) {
		this.amb_preference = s;
	}

	public boolean runInitial() {
		if(this.searcher == null || !this.searcher.isAlive()) {
			this.searcher = new Thread(()->{
				System.out.println("ControlSchemeManager: Beginning input search.");
				InputDevice[] devices = null, buff = null;
				for(;;) {
					int id = this.options.getSelected();
					if(id < 0) {
						devices = buff = null;
						int compat = -1;
						for(int i = 0; i < this.schemes.size(); i++) {
							buff = this.schemes.get(i).compatible(this.inputs);
							if(buff != null && buff.length > 0) {
								switch(this.amb_preference) {
									case PREFER_COMPLEX: {
										if(devices == null || buff.length > devices.length) {
											devices = buff;
											compat = i;
										}
										break;
									}
									case PREFER_SIMPLE: {
										if(devices == null || buff.length < devices.length) {
											devices = buff;
											compat = i;
										}
										break;
									}
									default:
									case NONE: {
										compat = (compat == -1) ? i : -2;
										devices = buff;
									}
								}
							}
						}
						if(compat >= 0) {
							this.schemes.get(compat).setup(devices);
							System.out.println("ControlSchemeManager: Set up control scheme [" + this.schemes.get(compat).getDesc() + "] with inputs:");
							for(InputDevice d : devices) {
								InputDevice.logDevice(d);
							}
							System.out.println();
							return;
						} else if(compat < -1) {
							System.out.println("ControlSchemeManager: Ambiguous case detected, please refine selection.");
						}
					} else {
						devices = this.schemes.get(id).compatible(this.inputs);
						if(devices != null && devices.length > 0) {
							this.schemes.get(id).setup(devices);
							System.out.println("ControlSchemeManager: Set up control scheme [" + this.schemes.get(id).getDesc() + "] with inputs:");
							for(InputDevice d : devices) {
								InputDevice.logDevice(d);
							}
							System.out.println();
							return;
						}
					}
					try{ Thread.sleep(500); }
					catch(InterruptedException e) { System.out.println(e.getMessage()); }
				}
			});
			this.searcher.start();
			return true;
		}
		return false;
	}
	public boolean runContinuous() {
		if(this.searcher == null || !this.searcher.isAlive()) {
			this.searcher = new Thread(()->{
				System.out.println("ControlSchemeManager: Beginning input search...");
				InputDevice[] devices = null, buff = null;
				int prev_selected = -1;
				int prev_active_id = -1;
				boolean has_any = false;
				for(;;) {
					int id = this.options.getSelected();
					if(!has_any || (prev_selected != id && prev_active_id != id)) {
						if(id < 0) {
							devices = buff = null;
							int compat = -1;
							for(int i = 0; i < this.schemes.size(); i++) {
								buff = this.schemes.get(i).compatible(this.inputs);
								if(buff != null && buff.length > 0) {
									switch(this.amb_preference) {
										case PREFER_COMPLEX: {
											if(devices == null || buff.length > devices.length) {
												devices = buff;
												compat = i;
											}
											break;
										}
										case PREFER_SIMPLE: {
											if(devices == null || buff.length < devices.length) {
												devices = buff;
												compat = i;
											}
											break;
										}
										default:
										case NONE: {
											compat = (compat == -1) ? i : -2;
											devices = buff;
										}
									}
								}
							}
							if(compat >= 0) {
								if(has_any) {
									this.schemes.get(prev_active_id).shutdown();
								}
								this.schemes.get(compat).setup(devices);
								System.out.println("ControlSchemeManager: Set up control scheme [" + this.schemes.get(compat).getDesc() + "] with inputs:");
								for(InputDevice d : devices) {
									InputDevice.logDevice(d);
								}
								System.out.println();
								prev_active_id = compat;
								prev_selected = id;
								has_any = true;
							} else if(compat < -1 && !has_any) {
								System.out.println("ControlSchemeManager: Ambiguous case detected, please refine selection.");
							}
						} else {
							devices = this.schemes.get(id).compatible(this.inputs);
							if(devices != null && devices.length > 0) {
								if(has_any) {
									this.schemes.get(prev_active_id).shutdown();
								}
								this.schemes.get(id).setup(devices);
								System.out.println("ControlSchemeManager: Set up control scheme [" + this.schemes.get(id).getDesc() + "] with inputs:");
								for(InputDevice d : devices) {
									InputDevice.logDevice(d);
								}
								System.out.println();
								prev_active_id = id;
								prev_selected = id;
								has_any = true;
							}
						}
					}
					try{ Thread.sleep(500); }
					catch(InterruptedException e) { System.out.println(e.getMessage()); }
				}
			});
			this.searcher.start();
			return true;
		}
		return false;
	}

}
