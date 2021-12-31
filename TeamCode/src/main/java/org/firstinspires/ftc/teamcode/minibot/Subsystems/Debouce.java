package org.firstinspires.ftc.teamcode.minibot.Subsystems;

public class Debouce {

    //public static long debounceTimeOut;

    public void debounce(int debounceTimeOut){
        try {
            Thread.sleep(debounceTimeOut);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
