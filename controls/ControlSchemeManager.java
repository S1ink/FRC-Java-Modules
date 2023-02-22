package frc.robot.team3407.controls;

import java.util.ArrayList;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.team3407.controls.Input.*;


public class ControlSchemeManager {

	public static interface ControlSchemeBase {
		public static interface Compat_F {
			public int[] test(InputDevice... inputs);
		}
		public static interface Setup_F {
			public void run(InputDevice... inputs);
		}
		
		public String getDesc();
		public int[] compatible(InputDevice... inputs);
		public void setup(InputDevice... inputs);
		default public void shutdown() {}
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
		public int[] compatible(InputDevice... inputs) {
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
	public static class CompatibilityTester implements ControlSchemeBase.Compat_F {
		private final InputMap[] requirements;

		public CompatibilityTester(InputMap... reqs) {
			this.requirements = reqs;
		}

		@Override
		public int[] test(InputDevice... inputs) {
			int[] ret = new int[this.requirements.length];
			int[] avail = {0, 1, 2, 3, 4, 5};	// 0 through (DriverStationJNI.kMaxJoysticks - 1)
			int found = 0;
			for(int i = 0; i < (avail.length - found); i++) {		// for each port that has not been claimed:
				int p = avail[found + i];
				for(int r = 0; r < (ret.length - found); r++) {		// for each remaining requirement:
					if(this.requirements[r].compatible(p)) {
						ret[found] = p;
						if(i != 0) {
							for(int q = i + found; q > found; q--) {
								avail[q] = avail[q - 1];
							}
						}
						found++;
						i--;
						break;
					}
				}
				if(found == ret.length) {
					return ret;
				}
			}
			return null;
		}

	}



	private final SendableChooser<Integer> options = new SendableChooser<>();
	private final ArrayList<ControlSchemeBase> schemes = new ArrayList<>();
	private final InputDevice[] inputs = new InputDevice[DriverStationJNI.kMaxJoysticks];
	private Thread searcher;

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
	public void publishSelector() { this.publishSelector("Control Scheme"); }
	public void publishSelector(String n) {
		SmartDashboard.putData(n, this.options);
	}

	public boolean runInitial() {
		if(this.searcher == null || !this.searcher.isAlive()) {
			this.searcher = new Thread(()->{
				System.out.println("Beginning input search thread.");
				InputDevice[] buff = null;
				int[] ports = null;
				for(;;) {
					int id = this.options.getSelected();
					if(id < 0) {
						for(ControlSchemeBase cs : this.schemes) {
							ports = cs.compatible(this.inputs);
							if(ports != null && ports.length > 0) {
								buff = new InputDevice[ports.length];
								for(int i = 0; i < ports.length; i++) {
									buff[i] = this.inputs[ports[i]];
								}
								cs.setup(buff);
								return;
							}
						}
					} else {
						ports = this.schemes.get(id).compatible(this.inputs);
						if(ports != null && ports.length > 0) {
							buff = new InputDevice[ports.length];
							for(int i = 0; i < ports.length; i++) {
								buff[i] = this.inputs[ports[i]];
							}
							this.schemes.get(id).setup(buff);
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
				System.out.println("Beginning input search thread.");
				InputDevice[] buff = null;
				int[] ports = null;
				int prev_selected = -1;
				int prev_active_id = -1;
				boolean has_any = false;
				for(;;) {
					int id = this.options.getSelected();
					if(!has_any || (has_any && prev_selected != id && id <= 0)) {
						if(id < 0) {
							for(int c = 0; c < this.schemes.size(); c++) {
								ports = this.schemes.get(c).compatible(this.inputs);
								if(ports != null && ports.length > 0) {
									buff = new InputDevice[ports.length];
									for(int i = 0; i < ports.length; i++) {
										buff[i] = this.inputs[ports[i]];
									}
									if(has_any) {
										this.schemes.get(prev_active_id).shutdown();
									}
									this.schemes.get(c).setup(buff);
									prev_active_id = c;
									prev_selected = c;
									has_any = true;
								}
							}
						} else {
							ports = this.schemes.get(id).compatible(this.inputs);
							if(ports != null && ports.length > 0) {
								buff = new InputDevice[ports.length];
								for(int i = 0; i < ports.length; i++) {
									buff[i] = this.inputs[ports[i]];
								}
								if(has_any) {
									this.schemes.get(prev_active_id).shutdown();
								}
								this.schemes.get(id).setup(buff);
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
