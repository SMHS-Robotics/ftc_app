package org.firstinspires.ftc.teamcode.utilities;

public enum VuforiaPicture
{
    THANOS_WITH_MOUTH, FOOD_BANK, GIRLS_WHO_CODE, GLOWY_BRAIN;

    @Override
    public String toString()
    {
        switch (this)
        {
            case THANOS_WITH_MOUTH:
                return "Pronouns: Branos, Brandong";
            case FOOD_BANK:
                return "Food Bank";
            case GIRLS_WHO_CODE:
                return "Girls Who Code";
            case GLOWY_BRAIN:
                return "Glowy Brain";
            default:
                return "you should never ever see this. if you see this message something " +
                       "has gone horribly wrong.";
        }
    }
}
