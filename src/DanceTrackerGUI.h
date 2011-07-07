#ifndef DANCETRACKERGUI_H
#define DANCETRACKERGUI_H

/* Include necessary MADTraC headers */
#include "MT_Core.h"
#include "MT_GUI.h"
#include "MT_Tracking.h"

#include "DanceTracker.h"

/**********************************************************************
 * GUI Frame Class
 *********************************************************************/

class DanceTrackerFrame : public MT_TrackerFrameBase
{
protected:
    DanceTracker* m_pDanceTracker;
    MT_Server* m_pServer;

    wxString m_sBackgroundImage;
    int m_iNToTrack;
    int m_iStartFrame;
    int m_iStopFrame;
    int m_iThreshFromCmdLine;
    wxString m_sNoteFromCommandLine;

public:
    DanceTrackerFrame(wxFrame* parent,
                         wxWindowID id = wxID_ANY,
                         const wxString& title = wxT("Tracker View"), 
                         const wxPoint& pos = wxDefaultPosition, 
                         const wxSize& size = wxSize(640,480),     
                         long style = MT_FIXED_SIZE_FRAME);

    virtual ~DanceTrackerFrame(){if(m_pServer) delete m_pServer;};

    void initTracker();
    void initUserData();

    void doUserStep();

    void handleCommandLineArguments(int argc, wxChar** argv);

    void onNewCapture();
};


/**********************************************************************
 * GUI App Class
 *********************************************************************/

class DanceTrackerApp
: public MT_AppBase
{
public:
    MT_FrameWithInit* createMainFrame()
    {
        return new DanceTrackerFrame(NULL);
    };
};


#endif /* DANCETRACKERGUI_H */
