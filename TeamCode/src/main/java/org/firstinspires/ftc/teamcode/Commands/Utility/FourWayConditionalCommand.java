package org.firstinspires.ftc.teamcode.Commands.Utility;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.BooleanSupplier;

/**
 * Four-Way Conditional Command Utility
 *
 * @author knutt5
 */
public class FourWayConditionalCommand extends CommandBase {

    private final Command m_Command1;
    private final Command m_Command2;
    private final Command m_Command3;
    private final Command m_CommandElse;
    private final BooleanSupplier m_Condition1;
    private final BooleanSupplier m_Condition2;
    private final BooleanSupplier m_Condition3;
    private Command m_selectedCommand;

    /**
     * Creates a Four-way conditional command to use in a command group
     *
     * @param condition1    the boolean condition to run command #1
     * @param command1      the command (or sequence) to run if condition #1 is met
     * @param condition2    the boolean condition to run command #2
     * @param command2      the command (or sequence) to run if condition #2 is met
     * @param condition3    the boolean condition to run command #3
     * @param command3      the command (or sequence) to run if condition #3 is met
     * @param commandElse   the command to run if no other condition is met
     */
    public FourWayConditionalCommand(BooleanSupplier condition1,
                                    Command command1,
                                    BooleanSupplier condition2,
                                    Command command2,
                                    BooleanSupplier condition3,
                                    Command command3,
                                    Command commandElse) {

        m_Command1 = command1;
        m_Command2 = command2;
        m_Command3 = command3;
        m_CommandElse = commandElse;
        m_Condition1 = condition1;
        m_Condition2 = condition2;
        m_Condition3 = condition3;
    }

    @Override
    public void initialize() {

        // determine which command to run given the conditions provided to us
        if (m_Condition1!=null && m_Condition1.getAsBoolean())
            m_selectedCommand = m_Command1;
        else if (m_Condition2!=null && m_Condition2.getAsBoolean())
            m_selectedCommand = m_Command2;
        else if (m_Condition3!=null && m_Condition3.getAsBoolean())
            m_selectedCommand = m_Command3;
        else
            m_selectedCommand = m_CommandElse;

        // not sure if this line will work as intended
        m_requirements.addAll(m_selectedCommand.getRequirements());

        // if selected command is not null, then initialize it
        if (m_selectedCommand!=null)
            m_selectedCommand.initialize();
    }

    @Override
    public void execute() {
        if (m_selectedCommand!=null)
            m_selectedCommand.execute();
    }

    @Override
    public boolean isFinished() {
        if (m_selectedCommand!=null)
            return m_selectedCommand.isFinished();
        else
            return true;
    }

    @Override
    public void end(boolean interrupted) {
        if (m_selectedCommand!=null)
            m_selectedCommand.end(interrupted);
    }

}