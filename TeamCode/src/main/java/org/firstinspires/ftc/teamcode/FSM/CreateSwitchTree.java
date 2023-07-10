import java.util.ArrayList;

public class CreateSwitchTree {
    public static ArrayList<String> FSM_STATE = new ArrayList<>();

    public static void main(String... args) {

        addState("IDLE");
        addState("STARTING");
        addState("DROP");
        addState("RETURN");
        addState("REBOUND");
        addState("END");
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