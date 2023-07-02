package org.firstinspires.ftc.teamcode.util.FSM;

import com.sun.tools.javac.code.Attribute;

import org.firstinspires.ftc.teamcode.hardware.actions.FollowTrajectorySequence;

import java.util.ArrayList;

public class CreateSwitchTree {
    public static ArrayList<String> FSM_STATE = new ArrayList<>();

    public static void main(String... args) {

        System.out.println(getSwitchTree());
    }

    public static void addState(String str) {
        FSM_STATE.add(str);
    }

    public static String getSwitchTree() {
        String treeStart = "switch(getState()) {\n";

        StringBuilder tree = new StringBuilder().append(treeStart);

        for (int i = 0; i < FSM_STATE.size(); i++) {
            tree.append("\tcase ").append('"').append(FSM_STATE.get(i)).append('"').append(":\n\t\tbreak;\n");
        }

        tree.append("}");

        return tree.toString();
    }
}