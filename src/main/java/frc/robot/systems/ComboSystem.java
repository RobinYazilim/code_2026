package frc.robot.systems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class ComboSystem {
    private final CommandPS4Controller controller;
    private final ArrayList<Combo> registeredCombos = new ArrayList<>();

    private final double expirationTime = 0.5;

    private static class Combo {
        public final List<String> sequence;
        public final Command action;
        public int currentIndex = 0;
        public double lastInputTime = 0;

        public Combo(List<String> sequence, Command action) {
            this.sequence = sequence;
            this.action = action;
        }
    }

    public ComboSystem(CommandPS4Controller controller)
    {
        this.controller = controller;
    }

    public void registerCommand(Command action, String... buttons) {
        registeredCombos.add(new Combo(Arrays.asList(buttons), action));
    }

    public void registerInputs()
    {
        controller.L1().onTrue(Commands.runOnce(() -> onInput("L1")));
        controller.R1().onTrue(Commands.runOnce(() -> onInput("R1")));
        controller.L2().onTrue(Commands.runOnce(() -> onInput("L2")));
        controller.R2().onTrue(Commands.runOnce(() -> onInput("R2")));
        controller.square().onTrue(Commands.runOnce(() -> onInput("Square")));
        controller.triangle().onTrue(Commands.runOnce(() -> onInput("Triangle")));
        controller.circle().onTrue(Commands.runOnce(() -> onInput("Circle")));
        controller.cross().onTrue(Commands.runOnce(() -> onInput("Cross")));
        controller.povUp().onTrue(Commands.runOnce(() -> onInput("Up")));
        controller.povDown().onTrue(Commands.runOnce(() -> onInput("Down")));
        controller.povLeft().onTrue(Commands.runOnce(() -> onInput("Left")));
        controller.povRight().onTrue(Commands.runOnce(() -> onInput("Right")));
    }

    public void onInput(String input) {
        double currentTime = Timer.getFPGATimestamp();

        for (Combo combo : registeredCombos) {
            if (combo.currentIndex > 0 && (currentTime - combo.lastInputTime > expirationTime)) {
                combo.currentIndex = 0;
            }

            if (combo.sequence.get(combo.currentIndex).equalsIgnoreCase(input)) {
                combo.currentIndex++;
                combo.lastInputTime = currentTime;

                if (combo.currentIndex == combo.sequence.size()) {
                    combo.action.schedule();
                    combo.currentIndex = 0;
                }
            } else {
                combo.currentIndex = 0;
            }
        }
    }

    
}
