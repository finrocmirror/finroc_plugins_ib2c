// this is a -*- C++ -*- file
//----------------------------------------------------------------------
/*!\file    tBehaviourDefinitions.h
 *
 * \author  Christopher Armbrust
 * \author  Bernd Helge Schaefer
 * \date    04-30-2007
 *
 * \brief   Contains definitions for the behaviours.
 *
 * \b tBehaviourDefinitions.h
 *
 * tBehaviourDefinitions.h contains definitions for the behaviours.
 *
 */
//----------------------------------------------------------------------

#ifndef _ibbc__tBehaviourDefinitions_h_
#define _ibbc__tBehaviourDefinitions_h_

class tBehaviourDefinitions
{

public:

  /*! enumeration type which contains a direction */
  //  _DESCR_(static, tBehaviourDefinitions, cDirectionNames, 5, Natural);
  enum tDIRECTION
  {
    eDIR_UNSPECIFIED,                   /*!< defining unspecified direction */
    eDIR_FORWARD,                       /*!< defining direction forward */
    eDIR_BACKWARD,                      /*!< defining direction backward */
    eDIR_ROTATION_LEFT,                 /*!< defining direction rotation left */
    eDIR_ROTATION_RIGHT,                /*!< defining direction rotation right */
    eDIR_SIDEWARD_LEFT,                 /*!< defining direction sideward left */
    eDIR_SIDEWARD_RIGHT,                /*!< defining direction sideward right */
    eDIR_FORWARD_BACKWARD,              /*!< defining direction forward or backward */
    eDIR_ROTATION_LEFT_ROTATION_RIGHT,  /*!< defining direction rotation left or rotation right */
    eDIR_SIDEWARD_LEFT_SIDEWARD_RIGHT,   /*!< defining direction sideward left or sideward right */
    eDIR_DIMENSION
  };

  /*! enumeration type which contains a region (i.e. one that is monitored by a sector map) */
  //  _DESCR_(static, tBehaviourDefinitions, cRegionNames, 5, Natural);
  enum tREGION
  {
    eREG_UNSPECIFIED,  /*!< defining unspecified region */
    eREG_FRONT,        /*!< defining front region */
    eREG_REAR,         /*!< defining rear region */
    eREG_FRONT_REAR,   /*!< defining front or rear region */
    eREG_LEFT,         /*!< defining left region */
    eREG_RIGHT,        /*!< defining right region */
    eREG_LEFT_RIGHT,    /*!< defining left or right region*/
    eREG_DIMENSION
  };


  /*!  enumeration type which contains the indices of the fusion methods */
  //  _DESCR_(static, tBehaviourDefinitions, cFusionMethodNames, 5, Natural);
  enum tFUSION_METHOD
  {
    eFUS_MAX,         /*!< Fusion method where the behaviour with the maximum activity wins. */
    eFUS_WEIGHT,      /*!< Fusion method weighting the outputs of the behaviours. */
    eFUS_WEIGHTED_SUM, /*!< Fusion method adding the outputs, weighted by their activity. */
    eFUS_DIMENSION
  };

  /*! enumeration which contains the different types of conditions that can be used by a conditional
      behaviour stimulation (\see mbbConditionalBehaviourStimulator); the values have been defined
      similar to the definition of Monica N. Nicolescu; extract from her homepage:
      - Permanent preconditions: preconditions that must be met during the entire execution of the
                                 behavior
      - Enabling preconditions: preconditions that must be met immediately before the activation of
                                a behavior
      - Ordering constraints: preconditions that must have been met at some point before the behavior
                              is activated
   */
  //  _DESCR_(static, tBehaviourDefinitions, cConditionTypeNames, 6, Natural);
  enum tCONDITION_TYPE
  {
    eCOND_ORDERING,   /*!< a condition that has to be fulfilled sometime before the stimulation */
    eCOND_ENABLING,   /*!< a condition that has to be fulfilled at the start of the stimulation */
    eCOND_PERMANENT,  /*!< a condition that has to be fulfilled during the entire time of the stimulation */
    eCOND_DIMENSION   /*!< end marker and dimension */
  };

  /*! enumeration which contains the different types of thresholds that can be used by a conditional
      behaviour stimulation (\see mbbConditionalBehaviourStimulator)
   */
  //  _DESCR_(static, tBehaviourDefinitions, cThresholdTypeNames, 11, Natural);
  enum tTHRESHOLD_TYPE
  {
    eTHRESHOLD_SMALLER,       /*!< value must be smaller than threshold */
    eTHRESHOLD_SMALLER_EQUAL, /*!< value must be smaller than or equal to threshold */
    eTHRESHOLD_EQUAL,         /*!< value must be equal to threshold */
    eTHRESHOLD_GREATER_EQUAL, /*!< value must be equal to or greater than threshold */
    eTHRESHOLD_GREATER,       /*!< value must be greater than threshold */
    eTHRESHOLD_DIMENSION      /*!< end marker and dimension */
  };

  /*! enumeration which contains different types of ports
   */
  //  _DESCR_(static, tBehaviourDefinitions, cPortTypeNames, 6, Natural);
  enum tPORT_TYPE
  {
    ePORT_SI,
    ePORT_SO,
    ePORT_CI,
    ePORT_CO,
    ePORT_PARAMETER,
    ePORT_DIMENSION
  };

};

#endif

