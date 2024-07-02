//
// Created by bkessler on 4/25/24.
//

#ifndef HATSCHET_SMASHINTEGERSCHEDULER_H
#define HATSCHET_SMASHINTEGERSCHEDULER_H

#if USE_Z3

#include "HatScheT/layers/IterativeModuloSchedulerLayer.h"
#include "HatScheT/base/Z3SchedulerBase.h"
#include "HatScheT/utility/Utility.h"

namespace HatScheT {

  class SMASHIntegerScheduler : public IterativeModuloSchedulerLayer, public Z3SchedulerBase {

  public:
    SMASHIntegerScheduler(Graph &g, ResourceModel &resourceModel, int II = -1);

  private:

    /*****************************
     * Main Scheduling Functions *
     *****************************/
    /*!
     * Function used to do stuff that has to be done before the II search loop starts. In this scheduler just used
     * for debugging prints and passing the set solver timeout to the z3-solver.
     */
    void scheduleInit() override;

    /*!
     * Main scheduling function is call from inside the II search loop in the iterative modulo scheduler layer.
     */
    void scheduleIteration() override;

    void generateModuloAndYVariables();

    map<Vertex*, z3::expr> mVariables;
    map<Vertex*, z3::expr> yVariables;

    void generateResourceVariables();

    map<Vertex*, z3::expr> rVariables;

    void boundYVariables();

    void boundMVariables();

    void boundRVariables();

    void addDependencyConstraints();

    void addHardwareConstraints();

    void parseSchedule();

    /*!
     * Bekanntes Dependency Constraint (als Erinnerung):
     * (1) t_oi − t_oj + L_oi - D_eij * II ≤ 0
     *
     * Für t_oi kann folgender Ausdruck eingesetzt werden. Somit spiegelt m_oi den Moduloslot wieder, in dem oi
     * gescheduled wird.
     * (2) t_oi = m_oi + (y_oi * II) | 0 <= m_oi < II
     *
     * Aus 1 und 2 entsteht für die Datenabhängigkeiten das folgende Constraint
     * (3) m_oi + (y_oi * II) - (m_oj + y_oj * II) + L_oi - D_eij * II ≤ 0 --> Welche Grenze für y?
     *
     * Hardwareinstanz, auf der die Operation oi ausgeführt wird.
     * (4) 0 <= r_oi < Anzahl der Hardwareinstanzen.
     *
     * Um Hardware-Constraints zu erfüllen, dürfen nie beide bedingungen (m_oi == m_oj) und (r_oi == r_oj) erfüllt sein.
     * (5) nand(m_oi == m_oj), (r_oi == r_oj)) für alle beschränkten Resourcen gleichen typs.
     */

  };

} // Hatschet

#endif

#endif //HATSCHET_SMASHINTEGERSCHEDULER_H
