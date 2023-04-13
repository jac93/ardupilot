
  struct Kgains  {

            float ang_roll_p;

            float kxp;

            float kxr;

            float kxp_int;

            float kxr_int;

            float kzp;

            float kzr;

            float kzp_int;

            float kzr_int;


        };

class LQRcontrolRY
{
    public:

        Kgains gains;
        void step(float rate_des[3],float rate_meas[3],float (&output)[2],float dt);
        void set_Klqi(Kgains gains);
        void initialize(void);
        void set_Kgains_default(Kgains gains);
        float Klqi[2][4] = { {0.4359,-0.0587,0.3794,-0.0668},
                            {0.1143,1.3073,0.0445,0.5691}
                            };
        float kp_roll_ = 1;
      




    private:
        void matrix_multiply(float A[2][4], float x[4], float u[2]);
        void update_integral_err(float (&error)[4],float dt);
        float err_rate[4] = {0,0,0,0};

        

};

