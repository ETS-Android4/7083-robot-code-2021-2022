package org.firstinspires.ftc.teamcode.subroutines;

import org.firstinspires.ftc.teamcode.hardware.Bot;

public class DeployBucket extends Thread {

    private Thread t;

    Bot b;

    DeployBucket(Bot b) {
        this.b = b;
    }

    @Override
    public void run() {
        // Tell the bucket to dump

    }

    public void start () {
        if (t == null) {
            t = new Thread (this, "bucketDeployment");
            t.start ();
        }
    }

}
