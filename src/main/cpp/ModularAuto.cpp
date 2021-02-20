#include <ModularAuto.h>
#include <wpi/raw_os_ostream.h>

ModularAuto::ModularAuto()
{
    int m_phaseNum = 0;
    bool m_phasedone = false;
    int m_phaseprog = 0;
}

void ModularAuto::AutoPhase()
{
    //wpi::errs() <<"Phase Progress:" << m_phaseprog;

    switch (m_phaseNum) //Each Case is one interchangable Autonomous Action Sequence
    {
    case 0: //goal start
    {
        switch (m_phaseprog)
        {
        case 0:
        {
            ControlVar.rspeed = MotionControl.rMotion.Drive(SensorVar.rdrvPos, positions_right[0][0]/m_TICKCONVERT, times[0][0], .1, 0.07);
            ControlVar.lspeed = MotionControl.lMotion.Drive(SensorVar.ldrvPos, positions_left[0][0]/m_TICKCONVERT, times[0][0], .1, 0.07);
            if (MotionControl.lMotion.GetDone() || MotionControl.rMotion.GetDone())
            {
                m_phaseprog++;
                m_t.Reset();
                m_t.Start();
                ControlVar.lspeed=0;
                ControlVar.rspeed=0;
            }
            break;
        }
        case 1:
        {
            ControlVar.shtspeed=1;
            if(m_t.Get()>=times[0][1])
            {
                m_phaseprog++;
                ControlVar.shtspeed=0;
                ControlVar.rspeed = MotionControl.rMotion.Drive(SensorVar.rdrvPos, positions_right[0][1], times[0][2], .1, 0.07);
                ControlVar.lspeed = MotionControl.lMotion.Drive(SensorVar.ldrvPos, positions_left[0][1], times[0][2], .1, 0.07);
            }
            break;
        }
        case 2:
        {
            ControlVar.rspeed = MotionControl.rMotion.Drive(SensorVar.rdrvPos, positions_right[0][1], times[0][2], .1, 0.07);
            ControlVar.lspeed = MotionControl.lMotion.Drive(SensorVar.ldrvPos, positions_left[0][1], times[0][2], .1, 0.07);
            if (MotionControl.lMotion.GetDone() || MotionControl.rMotion.GetDone())
                m_phaseprog++;
            break;
        }
        default:
        {
            ControlVar.rspeed = 0;
            ControlVar.lspeed = 0;
            m_phasedone = true;
            break;
        }
        }
        break;
    }
    
    case 1: //field start
    {  
        switch (m_phaseprog)
        {
            case 0:
            {
                
                ControlVar.rspeed = MotionControl.rMotion.Drive(SensorVar.rdrvPos, positions_right[1][0]/m_TICKCONVERT, times[1][0], .1, 0.07);
                ControlVar.lspeed = MotionControl.lMotion.Drive(SensorVar.ldrvPos, positions_left[1][0]/m_TICKCONVERT, times[1][0], .1, 0.07);
                if (MotionControl.lMotion.GetDone() || MotionControl.rMotion.GetDone())
                    m_phaseprog++;
                break;
            }
            case 1:
            {
                ControlVar.rspeed = MotionControl.rMotion.Drive(SensorVar.rdrvPos, positions_right[1][1]/m_TICKCONVERT, times[1][1], .1, 0.2);
                ControlVar.lspeed = MotionControl.lMotion.Drive(SensorVar.ldrvPos, positions_left[1][1]/m_TICKCONVERT, times[1][1], .1, 0.2);
                if (MotionControl.lMotion.GetDone() || MotionControl.rMotion.GetDone())
                    m_phaseprog++;
                break;
            }
            case 2:
            {
                ControlVar.rspeed = MotionControl.rMotion.Drive(SensorVar.rdrvPos, positions_right[1][2]/m_TICKCONVERT, times[1][2], .1, 0.07);
                ControlVar.lspeed = MotionControl.lMotion.Drive(SensorVar.ldrvPos, positions_left[1][2]/m_TICKCONVERT, times[1][2], .1, 0.07);
                if (MotionControl.lMotion.GetDone() || MotionControl.rMotion.GetDone())
                    m_phaseprog++;
                break;
            }
            case 3:
            {
                ControlVar.rspeed = MotionControl.rMotion.Drive(SensorVar.rdrvPos, positions_right[1][3]/m_TICKCONVERT, times[1][3], .1, 0.07);
                ControlVar.lspeed = MotionControl.lMotion.Drive(SensorVar.ldrvPos, positions_left[1][3]/m_TICKCONVERT, times[1][3], .1, 0.07);
                if (MotionControl.lMotion.GetDone() || MotionControl.rMotion.GetDone())
                    m_phaseprog++;
                break;
            }
        default:
        {
            ControlVar.rspeed = 0;
            ControlVar.lspeed = 0;
            m_phasedone = true;
            break;
        }
        }
        break;
    }

    case 2: //shield travel
    {
        switch (m_phaseprog)
        {
            case 0:
            {
                ControlVar.rspeed = MotionControl.rMotion.Drive(SensorVar.rdrvPos, m_TICKCONVERT*positions_right[2][0]+m_rencoffst, times[2][0], .1, 0.07);
                ControlVar.lspeed = MotionControl.lMotion.Drive(SensorVar.ldrvPos, m_TICKCONVERT*positions_left[2][0]+m_lencoffst, times[2][0], .1, 0.07);
                if (MotionControl.lMotion.GetDone() || MotionControl.rMotion.GetDone())
                m_phaseprog++;
                break;
            }
            case 1:
            {
                ControlVar.rspeed = MotionControl.rMotion.Drive(SensorVar.rdrvPos, m_TICKCONVERT*positions_right[2][1]+m_rencoffst, times[2][1], .1, 0.07);
                ControlVar.lspeed = MotionControl.lMotion.Drive(SensorVar.ldrvPos, m_TICKCONVERT*positions_left[2][1]+m_lencoffst, times[2][1], .1, 0.07);
                if (MotionControl.lMotion.GetDone() || MotionControl.rMotion.GetDone())
                m_phaseprog++;
                break;
            }
            case 2:
            {
                ControlVar.rspeed = MotionControl.rMotion.Drive(SensorVar.rdrvPos, m_TICKCONVERT*positions_right[2][2]+m_rencoffst, times[2][2], .1, 0.07);
                ControlVar.lspeed = MotionControl.lMotion.Drive(SensorVar.ldrvPos, m_TICKCONVERT*positions_left[2][2]+m_lencoffst, times[2][2], .1, 0.07);
                if (MotionControl.lMotion.GetDone() || MotionControl.rMotion.GetDone())
                    m_phaseprog++;
                break;
            }
            case 3:
            {
                ControlVar.rspeed = MotionControl.rMotion.Drive(SensorVar.rdrvPos, m_TICKCONVERT*positions_right[2][3]+m_rencoffst, times[2][3], .1, 0.07);
                ControlVar.lspeed = MotionControl.lMotion.Drive(SensorVar.ldrvPos, m_TICKCONVERT*positions_left[2][3]+m_lencoffst, times[2][3], .1, 0.07);
                if (MotionControl.lMotion.GetDone() || MotionControl.rMotion.GetDone())
                    m_phaseprog++;
                break;
            }
        default:
        {
            ControlVar.rspeed = 0;
            ControlVar.lspeed = 0;
            m_phasedone = true;
            break;
        }
        }
        break;
    }
    case 3: //trench travel
    {
        switch (m_phaseprog)
        {
        case 0:
        {
            ControlVar.rspeed = MotionControl.rMotion.Drive(SensorVar.rdrvPos, -m_TICKCONVERT*positions_right[3][0]+m_rencoffst, times[3][0], .1, 0.07);
            ControlVar.lspeed = MotionControl.lMotion.Drive(SensorVar.ldrvPos, m_TICKCONVERT*positions_left[3][0]+m_lencoffst, times[3][0], .1, 0.07);
            if (MotionControl.lMotion.GetDone() || MotionControl.rMotion.GetDone())
                m_phaseprog++;
            break;
        }
        case 1:
        {
            ControlVar.rspeed = MotionControl.rMotion.Drive(SensorVar.rdrvPos, m_TICKCONVERT*positions_right[3][1]+m_rencoffst, times[3][1], .1, 0.07);
            ControlVar.lspeed = MotionControl.lMotion.Drive(SensorVar.ldrvPos, m_TICKCONVERT*positions_left[3][1]+m_lencoffst, times[3][1], .1, 0.07);
            if (MotionControl.lMotion.GetDone() || MotionControl.rMotion.GetDone())
                m_phaseprog++;
            break;
        }
        case 2:
        {
            ControlVar.rspeed = MotionControl.rMotion.Drive(SensorVar.rdrvPos, m_TICKCONVERT*positions_right[3][2]+m_rencoffst, times[3][2], .1, 0.07);
            ControlVar.lspeed = MotionControl.lMotion.Drive(SensorVar.ldrvPos, m_TICKCONVERT*positions_left[3][2]+m_lencoffst, times[3][2], .1, 0.07);
            if (MotionControl.lMotion.GetDone() || MotionControl.rMotion.GetDone())
                m_phaseprog++;
            break;
        }
        case 3:
        {
            ControlVar.rspeed = MotionControl.rMotion.Drive(SensorVar.rdrvPos, m_TICKCONVERT*positions_right[3][3]+m_rencoffst, times[3][3], .1, 0.07);
            ControlVar.lspeed = MotionControl.lMotion.Drive(SensorVar.ldrvPos, m_TICKCONVERT*positions_left[3][3]+m_lencoffst, times[3][3], .1, 0.07);
            if (MotionControl.lMotion.GetDone() || MotionControl.rMotion.GetDone())
                m_phaseprog++;
            break;
        }
        default:
        {
            ControlVar.rspeed = 0;
            ControlVar.lspeed = 0;
            m_phasedone = true;
            break;
        }
        }
        break;
    }
    case 4:
    {
        switch(m_phaseprog)
        {
            case 0:
            {
                ControlVar.rspeed = MotionControl.rMotion.Drive(SensorVar.rdrvPos, m_TICKCONVERT*16.89, 2, .1, 0.35);
                ControlVar.lspeed = MotionControl.lMotion.Drive(SensorVar.ldrvPos, m_TICKCONVERT*-16.89, 2, .1, 0.35);
                if (MotionControl.lMotion.GetDone() || MotionControl.rMotion.GetDone())
                    {
                        m_phaseprog++;
                        ControlVar.rspeed=0;
                        ControlVar.lspeed=0;
                    }
                break;
            }
        }
        break;
    }
    //Infinite Recharge at Home
    case 5: // Barrel Race
    {
        switch(m_phaseprog)
        {
            case 0:
            {
                //ControlVar.rspeed = MotionControl.rMotion.Curve(SensorVar.rdrvPos,SensorVar.gangle,3,.1,0.35,m_TICKCONVERT*curve_positions_x[0][0]+m_rencoffst,)
            }
        }
        break;
    }
    case 6: // Slalom
    {
        switch(m_phaseprog)
        {
            case 0:
            {
                ControlVar.rspeed = MotionControl.rMotion.Curve(SensorVar.rdrvPos,SensorVar.ldrvPos,SensorVar.gangle,3,.1,.35,m_TICKCONVERT*curve_positions_x[1][0][0]+m_rencoffst,m_TICKCONVERT*curve_positions_x[1][0][1]+m_rencoffst,m_TICKCONVERT*curve_positions_x[1][0][2]+m_rencoffst,m_TICKCONVERT*curve_positions_x[1][0][3]+m_rencoffst,m_TICKCONVERT*curve_positions_y[1][0][0]+m_rencoffst,m_TICKCONVERT*curve_positions_y[1][0][1]+m_rencoffst,m_TICKCONVERT*curve_positions_y[1][0][2]+m_rencoffst,m_TICKCONVERT*curve_positions_y[1][0][3]+m_rencoffst);
                ControlVar.lspeed = MotionControl.lMotion.Curve(SensorVar.ldrvPos,SensorVar.rdrvPos,SensorVar.gangle,3,.1,.35,m_TICKCONVERT*curve_positions_x[1][0][0]+m_lencoffst,m_TICKCONVERT*curve_positions_x[1][0][1]+m_lencoffst,m_TICKCONVERT*curve_positions_x[1][0][2]+m_lencoffst,m_TICKCONVERT*curve_positions_x[1][0][3]+m_lencoffst,m_TICKCONVERT*curve_positions_y[1][0][0]+m_lencoffst,m_TICKCONVERT*curve_positions_y[1][0][1]+m_lencoffst,m_TICKCONVERT*curve_positions_y[1][0][2]+m_lencoffst,m_TICKCONVERT*curve_positions_y[1][0][3]+m_lencoffst);
                
                break;
            }
            case 1:
            {
                ControlVar.rspeed = MotionControl.rMotion.Curve(SensorVar.rdrvPos,SensorVar.ldrvPos,SensorVar.gangle,3,.1,.35,m_TICKCONVERT*curve_positions_x[1][1][0]+m_rencoffst,m_TICKCONVERT*curve_positions_x[1][1][1]+m_rencoffst,m_TICKCONVERT*curve_positions_x[1][1][2]+m_rencoffst,m_TICKCONVERT*curve_positions_x[1][1][3]+m_rencoffst,m_TICKCONVERT*curve_positions_y[1][1][0]+m_rencoffst,m_TICKCONVERT*curve_positions_y[1][1][1]+m_rencoffst,m_TICKCONVERT*curve_positions_y[1][1][2]+m_rencoffst,m_TICKCONVERT*curve_positions_y[1][1][3]+m_rencoffst);
                ControlVar.lspeed = MotionControl.lMotion.Curve(SensorVar.ldrvPos,SensorVar.rdrvPos,SensorVar.gangle,3,.1,.35,m_TICKCONVERT*curve_positions_x[1][1][0]+m_lencoffst,m_TICKCONVERT*curve_positions_x[1][1][1]+m_lencoffst,m_TICKCONVERT*curve_positions_x[1][1][2]+m_lencoffst,m_TICKCONVERT*curve_positions_x[1][1][3]+m_lencoffst,m_TICKCONVERT*curve_positions_y[1][1][0]+m_lencoffst,m_TICKCONVERT*curve_positions_y[1][1][1]+m_lencoffst,m_TICKCONVERT*curve_positions_y[1][1][2]+m_lencoffst,m_TICKCONVERT*curve_positions_y[1][1][3]+m_lencoffst);
                
                break;
            }
            case 2:
            {
                ControlVar.rspeed = MotionControl.rMotion.Curve(SensorVar.rdrvPos,SensorVar.ldrvPos,SensorVar.gangle,3,.1,.35,m_TICKCONVERT*curve_positions_x[1][2][0]+m_rencoffst,m_TICKCONVERT*curve_positions_x[1][2][1]+m_rencoffst,m_TICKCONVERT*curve_positions_x[1][2][2]+m_rencoffst,m_TICKCONVERT*curve_positions_x[1][0][3]+m_rencoffst,m_TICKCONVERT*curve_positions_y[1][2][0]+m_rencoffst,m_TICKCONVERT*curve_positions_y[1][2][1]+m_rencoffst,m_TICKCONVERT*curve_positions_y[1][2][2]+m_rencoffst,m_TICKCONVERT*curve_positions_y[1][2][3]+m_rencoffst);
                ControlVar.lspeed = MotionControl.lMotion.Curve(SensorVar.ldrvPos,SensorVar.rdrvPos,SensorVar.gangle,3,.1,.35,m_TICKCONVERT*curve_positions_x[1][2][0]+m_lencoffst,m_TICKCONVERT*curve_positions_x[1][2][1]+m_lencoffst,m_TICKCONVERT*curve_positions_x[1][2][2]+m_lencoffst,m_TICKCONVERT*curve_positions_x[1][0][3]+m_lencoffst,m_TICKCONVERT*curve_positions_y[1][2][0]+m_lencoffst,m_TICKCONVERT*curve_positions_y[1][2][1]+m_lencoffst,m_TICKCONVERT*curve_positions_y[1][2][2]+m_lencoffst,m_TICKCONVERT*curve_positions_y[1][2][3]+m_lencoffst);
                
                break;
            }
            case 3:
            {
                ControlVar.rspeed = MotionControl.rMotion.Curve(SensorVar.rdrvPos,SensorVar.ldrvPos,SensorVar.gangle,3,.1,.35,m_TICKCONVERT*curve_positions_x[1][3][0]+m_rencoffst,m_TICKCONVERT*curve_positions_x[1][3][1]+m_rencoffst,m_TICKCONVERT*curve_positions_x[1][3][2]+m_rencoffst,m_TICKCONVERT*curve_positions_x[1][3][3]+m_rencoffst,m_TICKCONVERT*curve_positions_y[1][3][0]+m_rencoffst,m_TICKCONVERT*curve_positions_y[1][3][1]+m_rencoffst,m_TICKCONVERT*curve_positions_y[1][3][2]+m_rencoffst,m_TICKCONVERT*curve_positions_y[1][3][3]+m_rencoffst);
                ControlVar.lspeed = MotionControl.lMotion.Curve(SensorVar.ldrvPos,SensorVar.rdrvPos,SensorVar.gangle,3,.1,.35,m_TICKCONVERT*curve_positions_x[1][3][0]+m_lencoffst,m_TICKCONVERT*curve_positions_x[1][3][1]+m_lencoffst,m_TICKCONVERT*curve_positions_x[1][3][2]+m_lencoffst,m_TICKCONVERT*curve_positions_x[1][3][3]+m_lencoffst,m_TICKCONVERT*curve_positions_y[1][3][0]+m_lencoffst,m_TICKCONVERT*curve_positions_y[1][3][1]+m_lencoffst,m_TICKCONVERT*curve_positions_y[1][3][2]+m_lencoffst,m_TICKCONVERT*curve_positions_y[1][3][3]+m_lencoffst);

                break;
            }
        }
        break;
    }
    default:
    {
        wpi::errs() << "Auto Phase-ID Not Recognized";
        ControlVar.rspeed = 0;
        ControlVar.lspeed = 0;
        break;
    }
    }
}

void ModularAuto::RunAutoPhase(int PhaseNum)
{
    m_phaseNum = PhaseNum;
    AutoPhase();
}

bool ModularAuto::GetPhaseDone()
{
    return m_phasedone;
}

void ModularAuto::AdvancePhase()
{
    m_phasedone = false;
    m_phaseprog = 0;
    m_rencoffst= SensorVar.rdrvPos;
    m_lencoffst= SensorVar.ldrvPos;
}
