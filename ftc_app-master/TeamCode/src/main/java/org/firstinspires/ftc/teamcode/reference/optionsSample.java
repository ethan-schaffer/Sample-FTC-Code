package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.options.*;
/**
 * Options menu sample OPMode implementation
 */

public class optionsSample extends OpMode {
    private OptionMenu menu;

    @Override
    public void init() {
        OptionMenu.Builder builder = new OptionMenu.Builder(hardwareMap.appContext);
        //Setup a SingleSelectCategory
        SingleSelectCategory alliance = new SingleSelectCategory("alliance");
        alliance.addOption("Red");
        alliance.addOption("Blue");
        builder.addCategory(alliance);
        //Setup a NumberCategory
        menu = builder.create();
        //Display menu
        menu.show();
    }

    @Override
    public void loop() {
        //Loops through each category, getting the input from menu
        for (Category c : menu.getCategories())
            telemetry.addData(c.getName(), menu.selectedOption(c.getName()));
    }
}