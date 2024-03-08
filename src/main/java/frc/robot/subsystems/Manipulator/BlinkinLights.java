package frc.robot.subsystems.Manipulator;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class BlinkinLights {
    private Spark lights;

    public BlinkinLights() {
        lights = new Spark(2);
    }

    public void setVoltage(double volts){
        lights.setVoltage(volts);
    }
    public void rainbowParty(){
        setVoltage(-.97);
    }
    public void zeroLights(){
        setVoltage(0);
    }
    public void white(){
        setVoltage(.93);
    }
    public void walkout(){
        setVoltage(-.71);
    }

}
