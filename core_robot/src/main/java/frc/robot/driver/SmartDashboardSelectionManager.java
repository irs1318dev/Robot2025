package frc.robot.driver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.lib.robotprovider.INetworkTableProvider;
import frc.lib.robotprovider.IRobotProvider;
import frc.lib.robotprovider.ISendableChooser;

@Singleton
public class SmartDashboardSelectionManager
{
    private final ISendableChooser<StartPosition> positionChooser;
    private final ISendableChooser<AutoRoutine> routineChooser;

    public enum StartPosition
    {
        Right,
        Center,
        Left,
        None,
    }

    public enum AutoRoutine
    {
        None,
        taxi,
        algaeProcess,
        relative,
        oneCoral,
        twoCoral,
        threeCoral,
        threeCoralV2,
        threeCoralWithTimeBasedFallback,
        twoCoralAngled,
        threeCoralAngled,
        test,
        netOneCoralOneAlgae,
        netOneCoralTwoAlgae,
    }

    /**
     * Initializes a new SmartDashboardSelectionManager
     */
    @Inject
    public SmartDashboardSelectionManager(
        IRobotProvider provider)
    {
        INetworkTableProvider networkTableProvider = provider.getNetworkTableProvider();

        // Routine Chooser
        this.routineChooser = networkTableProvider.getSendableChooser("Auto Routine");
        this.routineChooser.addDefault("None", AutoRoutine.None);
        this.routineChooser.addObject("Taxi", AutoRoutine.taxi);
        this.routineChooser.addObject("Relative", AutoRoutine.relative);
        // this.routineChooser.addObject("Algae Process", AutoRoutine.algaeProcess);
        this.routineChooser.addObject("Vision One Coral", AutoRoutine.oneCoral);
        this.routineChooser.addObject("Vision Two Coral", AutoRoutine.twoCoral);
        this.routineChooser.addObject("Vision Three Coral", AutoRoutine. threeCoral);
        this.routineChooser.addObject("Vision Three Coral V2", AutoRoutine. threeCoralV2);
        this.routineChooser.addObject("Vision Three Coral with Time Based Fallback", AutoRoutine.threeCoralWithTimeBasedFallback);
        // this.routineChooser.addObject("Vision Two Coral Angled", AutoRoutine.twoCoralAngled);
        // this.routineChooser.addObject("Vision Three Coral Angled", AutoRoutine.threeCoralAngled);
        this.routineChooser.addObject("Net 1 Coral and 1 Algae", AutoRoutine.netOneCoralOneAlgae);
        // this.routineChooser.addObject("Net 1 Coral and 2 Algae", AutoRoutine.netOneCoralTwoAlgae);
        // this.routineChooser.addObject("Test", AutoRoutine.test);

        // Position Chooser
        this.positionChooser = networkTableProvider.getSendableChooser("Start Position");
        this.positionChooser.addDefault("None", StartPosition.None);
        this.positionChooser.addObject("Right", StartPosition.Right);
        this.positionChooser.addObject("Left", StartPosition.Left);
        this.positionChooser.addObject("Center", StartPosition.Center);
    }

    public StartPosition getSelectedStartPosition()
    {
        return SmartDashboardSelectionManager.GetSelectedOrDefault(this.positionChooser, StartPosition.None);
    }

    public AutoRoutine getSelectedAutoRoutine()
    {
        return SmartDashboardSelectionManager.GetSelectedOrDefault(this.routineChooser, AutoRoutine.None);
    }

    private static <T> T GetSelectedOrDefault(ISendableChooser<T> chooser, T defaultValue)
    {
        T selected = chooser.getSelected();
        if (selected == null)
        {
            selected = defaultValue;
        }

        return selected;
    }
}
