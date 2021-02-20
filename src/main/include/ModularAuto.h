#pragma once

#include "Motion.h"
#include <vector>
#include <frc/Timer.h>

class ModularAuto
{
    private:
        bool m_phasedone;
        int m_phaseNum;
        int m_phaseprog;
        double m_lencoffst;
        double m_rencoffst;
        const double m_TICKCONVERT= MotionControl.rMotion.ConvertInchTick(6,1,12.75);//1.4784
        std::vector<std::vector<double>> times = 
        {    
            {3.5, 1.5, 3.5},
            {6, 1, 4, 4},
            {4, 4, 4, 4},
            {4, 4, 4, 8}
        };
        std::vector<std::vector<double>> positions_right = 
        {    
            {120, 0},
            {209.84, 176.1, 276.86, 176.1},
            {16.89, 164.2, 185.3, 484.91},
            {-16.89, 55.077, 44.2, 409.1}
        };
        std::vector<std::vector<double>> positions_left = 
        {    
            {120, 0},
            {209.84, 243.6, 344.34, 243.6},
            {-16.89, 130.4, 109.3, 408.92},
            {16.89, 88.857, 105.7, 476.7}
        };

        std::vector<std::vector<std::vector<double>>> curve_positions_x =
        {
            {{0,1,2,3}},
            {{0,0,80,150},{150,270,270,300},{300,270,270,150},{150,80,0,0}}//slalom
        };
        std::vector<std::vector<std::vector<double>>> curve_positions_y =
        {
            {{0,1,2,3}},
            {{0,0,60,60},{60,60,-40,30},{30,80,0,0},{0,0,60,60}}//slalom
        };
        frc::Timer m_t;
        //{4,4};
        //{6,4,4,4};
        //{4,4,4,4};
        //{4,4,4,4,8};
    public:
        ModularAuto();
        
        void AutoPhase();
        
        void RunAutoPhase(int PhaseNum);

        bool GetPhaseDone();

        void AdvancePhase();

        struct {double lspeed; double rspeed; double shtspeed;}ControlVar;
        struct {double rdrvPos; double ldrvPos; double gangle;}SensorVar;
        struct {Motion lMotion = Motion(true,0.02); Motion rMotion = Motion(false,0.02);}MotionControl;

};
