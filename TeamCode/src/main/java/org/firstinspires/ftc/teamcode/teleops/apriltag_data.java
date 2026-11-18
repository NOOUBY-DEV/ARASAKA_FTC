package org.firstinspires.ftc.teamcode.teleops;


public class apriltag_data
{
        public int id;
        public double tx;
        public double ty;
        public boolean is_valid;

        public apriltag_data(int id, double tx, double ty, boolean is_valid)
        {
            this.id = id;
            this.tx = tx;
            this.ty = ty;
            this.is_valid = is_valid;
        }

        public double get_tx()
        {
                return tx;
        }

        public Boolean is_red_team()
        {
                if (id < 30 || id > 42) return null;

                return id <= 37;
        }
}
