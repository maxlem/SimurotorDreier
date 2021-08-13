/*
 * Rotor.h
 *
 *  Created on: 14-Jul-2008
 *      Author: malem303
 */

#ifndef ROTOR_H_
#define ROTOR_H_
#include "IndividualElement.h"
#include "DegreeOfFreedom.h"
#include "Constants.h"

enum HubType {FULLY_ATRICULATED, TEETERING, DELTA_3, OFFSET_HINGE};
class Rotor : public IndividualElement {
private:

	int numberOfBlades;

	/*! \f$ r_{rotor} \f$ */
	double radius;

	/*! \f$ c \f$ */
	double chord;


    /*!\f$  \overline{m} \f$ */
    double linearMass;

    /*! \f$ \Omega_{rotor} \f$ */
    double rotorVelocity;

    /*! \f$ \theta_{\Delta_{3}} \f$ */
    double delta3Angle;

    HubType hubType;

    double preconeAngle;

    /*! \f$ \alpha_{0} \f$ */
    double liftCurveSlope;


    vector<double> dragCoefficientParameters;

    /*! \f$ h_{offset} \f$ (a.k.a "e")*/
    double horizontalHubOffset;

    /*! \f$ v_{offset} \f$ (a.k.a "h") */
    double verticalHubOffset;

    /*! \f$ k_{Hub} \f$ */
    double hubSpringConstant;


    /*! Cyclic function */

    /*! \f$ \theta_{collective} \f$ */
    DegreeOfFreedom collectivePitch;

    /*! \f$ \theta_{twist} \f$ */
    double twistAngle;

    /*! \f$ \theta_{swash_{side}} \f$ */
    DegreeOfFreedom lateralSwashplateAngle;

    /*! \f$ \theta_{swash_{front}} \f$ */
    DegreeOfFreedom longitudinalSwashplateAngle;


    /*! Flapping function */

    /*! \f$ \theta_{flap_{cone}} \f$ */
    double coningAngle;

    /*! \f$ \theta_{flap_{front}} \f$ */
    double longitudinalFlappingAngle;

    /*! \f$ \theta_{flap_{side}} \f$ */
    double lateralFlappingAngle;




    /*! \f$ power \f$ */
    double power;

    /*! \f$ \overline{\omega_{flap}}=\frac{\omega_{flap}}{\Omega_{tot}} \f$ */
//    double naturalFlappingFrequency;

    double computeInfluenceFactor(IndividualElement * influencedElement);

	/*!
	 * @param thrustForceFactor
	 * @return \f$ \epsilon_{0}=\overline{\frac{\overline{C_{D}}}{a_{0}}} \f$ the average drag coefficient for all azimuth values divided by the lift curve slope
	 */
	double computeEpsilon0(double thrustForceFactor) const;

    /*!
    * @param velocity \f$ \mathbf{v} \f$
    * @param tangentialVelocity \f$ \left\Vert \mathbf{v}_{tan}\right\Vert  \f$
    * @return \f$ \lambda=\frac{w}{\left\Vert \mathbf{v}_{tan}\right\Vert } \f$
    */
	double computeInflowRatio(vector<double> velocity, double tangentialVelocity) const;


    /*!
    * @param velocity \f$ \mathbf{v} \f$
    * @param tangentialVelocity \f$ \left\Vert \mathbf{v}_{tan}\right\Vert  \f$
    * @return \f$ \mu=\frac{u}{\left\Vert \mathbf{v}_{tan}\right\Vert } \f$
    */
   double computeAdvanceRatio(vector<double> velocity, double tangentialVelocity) const;

	/*!
	 *
	 * @param thrustForceFactor \f$ \overline{f_{T}} \f$ the non-dimensionnal thrust force factor
	 * @return \f$ B_{T} \f$ the tip loss factor
	 */
	double computeTipLossFactor(double thrustForceFactor) const;


    /*! \f$ \gamma \f$ */
    double computeLockNumber(double density) const;

    /*!
     * @return \f$ \sigma = \frac{b\cdot c\cdot }{\pi r_{rotor}} \f$ (p.198)
     */
    double computeSolidity() const;

    double computeNormalizedLiftCoefficient(double thrustForceFactor) const;

    double computeNormalizedDragCoefficient(double thrustForceFactor) const;
    /*!
     * \f$ m_{s}=\int_{0}^{r_{rotor}}s\cdot\overline{m}\cdot ds \f$
     * @return \f$ m_{s} \f$
     */
    double computeBladeFisrtMassMoment() const;

    /*!
     * \f$  m_{s^{2}}=\int_{0}^{r_{rotor}}s^{2}\cdot\overline{m}\cdot ds \f$
     * @return \f$ m_{s^{2}} \f$
     */
    double computeBladeInertialMoment() const;



	double computeAerodynamicTangentialVelocity(vector<double> aerodynamicAngularVelocity, double distanceFromRoot) const;

    /*!
     *
     * @return \f$ \Omega_{a}=\Omega-r_{a} \f$
     */
    double computeAerodynamicRotorVelocity(vector<double> aerodynamicAngularVelocity) const;

    /*!
     *
     * @return \f$ \Omega_{i}=\Omega-r_{i} \f$
     */
    double computeInertialRotorVelocity(vector<double> inertialAngularVelocity) const;

    /*!
    *
    * @param azimuth \f$ \theta_{a} \f$ the angular position of the rotating system ("\f$ Hub \f$")
    * @param distanceFromRoot \f$ s \f$ the distance between the blade element and the root of the blade
    * @return \f$ \theta_{flap} \f$ the angle of the tip path plane at azimuth \c azimuth \c
    */
   double computeflappingFunction(double azimuth, double distanceFromRoot) const;

   /*! @return \f$ \overline{\omega_{flap}}=\frac{\omega_{flap}}{\Omega_{rotor}}=\sqrt{\left(1+\frac{3\cdot h_{offset}}{2\left(r_{rotor}-h_{offset}\right)}+\frac{k_{hub}}{m_{s^{2}}\cdot\Omega_{rotor}^{2}}+\frac{\gamma}{8}\tan\left(\theta_{Delta_{3}}\right)\right)} \f$ */
   double computeNaturalFlappingFrequency(double lockNumber, double bladeInertialMoment, double rotorVelocity) const;

   /*!
       *
       * @param advanceRatio \f$ \mu \f$
       * @param lockNumber \f$ \gamma \f$
       * @param tipLossFactor	\f$ B_{T} \f$
       * @param epsilon0 \f$ \epsilon_{0} \f$
       * @param normalizedNaturalFlappingFrequency \f$ \overline{\omega}_{flap} \f$
       * @param normalizedRotorVelocity \f$ \overline{\Omega} \f$
       * @return \f$ \mathbf{M}_{flap}=\frac{\gamma\overline{\Omega}^{2}}{2}\left[\begin{array}{ccc}
  						\frac{\left(\overline{\omega}_{flap}^{2}\right)}{\frac{\gamma\overline{\Omega}^{2}}{2}} & \left(\frac{\mu B_{T}^{3}}{6\overline{\Omega}}\right)\left(1+\epsilon_{0}\right)\left(1-\overline{\Omega}\right) & 0\\
  						0 & \left(\frac{B_{T}^{4}}{4\overline{\Omega}}-\frac{B_{T}^{2}\mu^{2}}{8}\right) & \frac{\left(1-\overline{\omega}_{flap}^{2}\right)}{\frac{\gamma\overline{\Omega}^{2}}{2}}\\
  						\left(\frac{\mu B_{T}^{3}}{3}\right)\left(1+\epsilon_{0}\right) & \frac{\left(1-\overline{\omega}_{flap}^{2}\right)}{\frac{\gamma\overline{\Omega}^{2}}{2}} & -\left(\frac{B_{T}^{4}}{4}+\frac{B_{T}^{2}\mu^{2}}{8}\right)\end{array}\right]\f$
       */
      matrix<double> buildFlappingMatrix(double advanceRatio, double lockNumber, double tipLossFactor, double epsilon0, double normalizedNaturalFlappingFrequency, double normalizedRotorVelocity) const;
      /*!
        *
        * @param advanceRatio \f$ \mu \f$
        * @param lockNumber \f$ \gamma \f$
        * @param tipLossFactor	\f$ B_{T} \f$
        * @param normalizedNaturalFlappingFrequency \f$ \overline{\omega}_{flap} \f$
        * @param normalizedRotorVelocity \f$ \overline{\Omega} \f$
        * @return \f$ \mathbf{M}_{swash}=\frac{\gamma\overline{\Omega}^{2}}{2}\left[\begin{array}{cccc}
  							\left(\frac{B_{T}^{4}}{4}+\frac{B_{T}^{2}\mu^{2}}{4}\right) & \left(\frac{B_{T}^{5}}{5}+\frac{B_{T}^{3}\mu^{2}}{6}\right) & 0 & \left(\frac{\mu B_{T}^{3}}{3}\right)\\
  							\left(\frac{2B_{T}^{3}\mu}{3}\right) & \left(\frac{B_{T}^{4}\mu}{2}\right) & 0 & \left(\frac{B_{T}^{4}}{4}+\frac{3B_{T}^{2}\mu^{2}}{8}\right)\\
  							0 & 0 & \left(\frac{B_{T}^{4}}{4}+\frac{B_{T}^{2}\mu^{2}}{8}\right) & 0\end{array}\right]\f$
        */
      matrix<double> buildSwashplateMatrix(double advanceRatio, double lockNumber, double tipLossFactor, double normalizedRotorVelocity) const;

      /*!
  	*
  	* @param advanceRatio \f$ \mu \f$
  	* @param lockNumber \f$ \gamma \f$
  	* @param tipLossFactor	\f$ B_{T} \f$
  	* @param epsilon0 \f$ \epsilon_{0} \f$
  	* @return \f$ \mathbf{M}_{ratios}=\frac{\gamma\overline{\Omega}^{2}\left(1+\epsilon_{0}\right)}{2}\left[\begin{array}{ccc}
  						\left(\frac{B_{T}^{3}}{3}\right) & \left(\frac{\mu B_{T}^{3}}{6}\right) & 0\\
  						\left(\frac{\mu B_{T}^{2}}{2}-\frac{\mu^{3}}{8}\right) & \left(\frac{B_{T}^{4}}{4}\right) & 0\\
  						0 & 0 & \left(\frac{B_{T}^{4}}{4}\right)\end{array}\right]\f$
      */
     matrix<double> buildRatioMatrix(double advanceRatio, double lockNumber, double tipLossFactor, double epsilon0, double normalizedRotorVelocity) const;

  	/*!
  	* @param bladeFirstMassMoment \f$ m_{s} \f$
  	* @param bladeInertialMassMoment \f$ m_{s^{2}}\f$
  	* @param coningAngle \f$ \theta_{flap_{cone}} \f$
  	* @param lateralFlappingAngle \f$  \theta_{flap_{side}} \f$
  	* @param longitudinalFlappingAngle \f$ \theta_{flap_{front}} \f$
  	* @return \f$ \mathbf{M}_{\mathbf{a}}=\left[\begin{array}{ccccccc}
  					0 & 0 & 0 & 0 & -\frac{m_{s}r_{rotor}\theta_{flap_{front}}}{2m_{s^{2}}} & \frac{m_{s}r_{rotor}\theta_{flap_{side}}}{2m_{s^{2}}} & \frac{m_{s}r_{rotor}}{m_{s^{2}}}\\
  					0 & -2 & 1 & 0 & 0 & \frac{m_{s}r_{rotor}\theta_{flap_{cone}}}{m_{s^{2}}} & 0\\
  					2 & 0 & 0 & 1 & \frac{m_{s}r_{rotor}\theta_{flap_{cone}}}{m_{s^{2}}} & 0 & 0\end{array}\right]\f$
  	*/
  	matrix<double> buildAccelerationMatrix(double bladeFirstMassMoment, double bladeInertialMassMoment, vector<double> flappingAngles) const;


  	vector<double> buildAccelerationVector(double normalizedInertialRollVelocity, double normalizedInertialPitchVelocity, double inertialRotorVelocity, vector<double> inertialVelocityDerivative, vector<double> inertialAngularVelocityDerivative) const;

public:

	/*!
	 * Compute this rotor's wash on another IndividualElement
	 * @param influencedElement the other IndividualElement
	 * @param density the current value for \f$ \rho \f$
	 * @return \f$ \mathbf{v}_{wash} \f$ the external wash
	 */
	vector<double> computeExternalWashInfluence(IndividualElement * influencedElement, double density);

	/*!
	 *
	 * @param advanceRatio  \f$ \mu \f$
	 * @param aerodynamicInflowRatio \f$ \lambda_{a} \f$
	 * @param tipLossFactor \f$ B_{T} \f$
	 * @param epsilon0 \f$ \epsilon_{0} \f$
	 * @param swashplateAngles \f$ \left[\begin{array}{c}
				\theta_{collective}\\
				\theta_{twist}\\
				\theta_{swash_{side}}\\
				\theta_{swash_{front}}\end{array}\right]\f$
	 * @param flappingAngles \f$ \left[\begin{array}{c}
				\theta_{flap_{cone}}\\
				\theta_{flap_{front}}\\
				\theta_{flap_{side}}\end{array}\right]\f$
	 * @param normalizedAerodynamicRollVelocity \f$ \overline{p} \f$
	 * @param normalizedAerodynamicPitchVelocity \f$ \overline{q} \f$
	 * @return \f$ \overline{f_{x}}=\frac{\sigma\cdot a_{0}}{2}\left(\begin{array}{c}
			-\frac{B_{T}^{2}\mu}{2}\epsilon_{0}+\left(\frac{B_{T}\mu}{2}\lambda_{a}-\frac{B_{T}^{3}}{3}\theta_{flap_{front}}\right)\theta_{collective}\ldots\\
			\ldots+\left(\frac{B_{T}^{2}\mu}{4}\lambda_{a}-\frac{B_{T}^{4}}{4}\theta_{flap_{front}}\right)\theta_{twist}+\left(\frac{B_{T}^{3}}{6}\theta_{flap_{cone}}\right)\left(\theta_{swash_{side}}+\theta_{flap_{side}}\right)\\
			\ldots+\left(\frac{B_{T}^{2}}{4}\lambda_{a}-\frac{B_{T}^{2}\mu}{4}\theta_{flap_{front}}\right)\theta_{swash_{front}}-\frac{3B_{T^{2}}}{4}\lambda_{a}\theta_{flap_{front}}-\frac{B_{T}^{2}}{4}\left(\theta_{flap_{cone}}^{2}+\theta_{flap_{front}}^{2}\right)\mu\\
			\ldots+\left(\frac{B_{T}^{3}}{6}\theta_{collective}+\frac{B_{T}^{4}}{8}\theta_{twist}+\frac{B_{T}^{2}}{2}\lambda_{a}+\frac{3B_{T}^{2}\mu}{16}\theta_{swash_{front}}+\frac{B_{T}^{2}\mu}{16}\theta_{flap_{front}}\right)\overline{p_{a}}\\
			\ldots+\left(\frac{B_{T}^{3}}{6}\theta_{flap_{cone}}+\frac{B_{T}^{2}\mu}{16}\theta_{swash_{side}}+\frac{B_{T}^{2}\mu}{16}\theta_{flap_{side}}\right)\overline{q_{a}}\end{array}\right)\f$
	 */
	double computeXForceFactor(double advanceRatio, double aerodynamicInflowRatio, double tipLossFactor, double epsilon0, vector<double> swashplateAngles, vector<double> flappingAngles, double normalizedAerodynamicRollVelocity, double normalizedAerodynamicPitchVelocity) const;

	/*!
	 *
	 * @param advanceRatio \f$ \mu \f$
	 * @param aerodynamicInflowRatio \f$ \lambda_{a} \f$
	 * @param tipLossFactor \f$ B_{T} \f$
	 * @param epsilon0 \f$ \epsilon_{0} \f$
	 * @param swashplateAngles \f$ \left[\begin{array}{c}
				\theta_{collective}\\
				\theta_{twist}\\
				\theta_{swash_{side}}\\
				\theta_{swash_{front}}\end{array}\right]\f$
	 * @param flappingAngles \f$ \left[\begin{array}{c}
				\theta_{flap_{cone}}\\
				\theta_{flap_{front}}\\
				\theta_{flap_{side}}\end{array}\right]\f$
	 * @param normalizedAerodynamicRollVelocity \f$ \overline{p} \f$
	 * @param normalizedAerodynamicPitchVelocity \f$ \overline{q} \f$
	 * @return \f$ \overline{f_{y}}=\frac{\sigma\cdot a_{0}}{2}\left(\begin{array}{c}
				\left(\left(\frac{B_{T}^{3}}{3}+\frac{B_{T}\mu^{2}}{2}\right)\theta_{flap_{side}}-\frac{3B_{T}^{2}\mu}{4}\theta_{flap_{cone}}\right)\theta_{collective}\ldots\\
				\ldots+\left(\left(\frac{B_{T}^{4}}{4}+\frac{B_{T}^{2}\mu^{2}}{4}\right)\theta_{flap_{side}}-\frac{B_{T}^{3}\mu}{2}\theta_{flap_{cone}}\right)\theta_{twist}\\
				\ldots+\left(\frac{B_{T}^{2}}{4}\lambda_{a}+\frac{B_{T}^{2}\mu}{4}\theta_{flap_{front}}\right)\theta_{swash_{side}}\\
				\ldots+\left(-\left(\frac{B_{T}^{3}}{6}+\frac{B_{T}\mu^{2}}{2}\right)\theta_{flap_{cone}}+\frac{B_{T}^{2}\mu}{2}\theta_{flap_{side}}\right)\theta_{swash_{front}}\\
				\ldots+\left(\frac{3B_{T}^{2}}{4}\theta_{flap_{side}}-\frac{3B_{T}\mu}{2}\theta_{flap_{cone}}\right)\lambda_{a}\\
				\ldots+\left(\frac{B_{T}^{2}}{6}\theta_{flap_{cone}}+\frac{B_{T}^{2}\mu}{4}\theta_{flap_{side}}-B_{T}\mu^{2}\theta_{flap_{cone}}\right)\theta_{flap_{front}}\\
				\ldots+\left(\frac{B_{T}^{2}\mu}{16}\theta_{swash_{side}}-\frac{B_{T}^{3}}{6}\theta_{flap_{cone}}+\frac{5B_{T}^{2}\mu}{16}\theta_{flap_{side}}\right)\overline{p_{a}}\\
				\ldots+\left(\frac{B_{T}^{3}}{6}\theta_{collective}+\frac{B_{T}^{4}}{8}\theta_{twist}+\frac{B_{T}^{2}}{2}\lambda_{a}+\frac{B_{T}^{2}\mu}{16}\theta_{swash_{front}}+\frac{7B_{T}^{2}\mu}{16}\theta_{flap_{front}}\right)\overline{q_{a}}\end{array}\right)
				\f$
	 */
    double computeYForceFactor(double advanceRatio, double aerodynamicInflowRatio, double tipLossFactor, double epsilon0, vector<double> swashplateAngles, vector<double> flappingAngles, double normalizedAerodynamicRollVelocity, double normalizedAerodynamicPitchVelocity) const;


    /*!
     *
	 * @param advanceRatio \f$ \mu \f$
	 * @param aerodynamicInflowRatio \f$ \lambda_{a} \f$
	 * @param tipLossFactor \f$ B_{T} \f$
	 * @param epsilon0 \f$ \epsilon_{0} \f$
	 * @param swashplateAngles \f$ \left[\begin{array}{c}
				\theta_{collective}\\
				\theta_{twist}\\
				\theta_{swash_{side}}\\
				\theta_{swash_{front}}\end{array}\right]\f$
	 * @param normalizedAerodynamicRollVelocity \f$ \overline{p} \f$
     * @return \f$ \overline{f_{z}}=-\frac{\sigma\cdot a_{0}}{2}\left(\begin{array}{c}
				\left(\frac{B_{T}^{3}}{3}+\frac{\mu^{2}B_{T}}{2}\right)\theta_{collective}+\left(\frac{B_{T}^{4}}{4}+\frac{\mu^{2}B_{T}^{2}}{4}\right)\theta_{twist}\ldots\\
				\ldots+\left(\frac{B_{T}^{2}}{2}+\frac{\mu^{2}}{4}\right)\left(1+\epsilon_{0}\right)\lambda_{a}+\left(\frac{\mu B_{T}^{2}}{2}+\frac{\mu^{3}}{8}\right)\theta_{swash_{front}}\\
				\ldots+\left(\frac{B_{T}^{2}\mu}{4}\left(1+\epsilon_{0}\right)\right)\overline{p_{a}}\end{array}\right)
				\f$
     */
    double computeZForceFactor(double advanceRatio, double inflowRatio, double tipLossFactor, double epsilon0, vector<double> swashplateAngles, double normalizedAerodynamicRollVelocity) const;

    /*!
     *
	 * @param advanceRatio \f$ \mu \f$
	 * @param aerodynamicInflowRatio \f$ \lambda_{a} \f$
	 * @param tipLossFactor \f$ B_{T} \f$
	 * @param epsilon0 \f$ \epsilon_{0} \f$
	 * @param swashplateAngles \f$ \left[\begin{array}{c}
				\theta_{collective}\\
				\theta_{twist}\\
				\theta_{swash_{side}}\\
				\theta_{swash_{front}}\end{array}\right]\f$
	 * @param flappingAngles \f$ \left[\begin{array}{c}
				\theta_{flap_{cone}}\\
				\theta_{flap_{front}}\\
				\theta_{flap_{side}}\end{array}\right]\f$
	 * @param normalizedAerodynamicRollVelocity \f$ \overline{p} \f$
     * @return \f$\overline{f_{a_{x}|IERA}^{\curvearrowright}}=-\frac{\sigma\cdot a_{0}}{2}\left(\begin{array}{c}
				\left(\frac{B_{T}^{3}\mu}{3}\right)\theta_{collective}+\left(\frac{B_{T}^{4}\mu}{4}\right)\theta_{twist}+\left(\frac{B_{T}^{2}\mu}{4}\right)\lambda_{a}+\left(\frac{B_{T}^{4}\mu}{8}\right)\overline{p_{a}}\ldots\\
				\ldots+\left(\frac{B_{T}^{4}}{8}+\frac{3B_{T}^{2}\mu^{2}}{16}\right)\theta_{swash_{front}}-\left(\frac{B_{T}^{4}}{8}+\frac{B_{T}^{2}\mu^{2}}{16}\right)\theta_{flap_{front}}\end{array}\right)
				\f$
     */
    double computeXMomentFactor(double advanceRatio, double aerodynamicInflowRatio, double tipLossFactor, double epsilon0, vector<double> swashplateAngles, vector<double> flappingAngles, double normalizedAerodynamicRollVelocity) const;

    /*!
     *
	 * @param advanceRatio \f$ \mu \f$
	 * @param aerodynamicInflowRatio \f$ \lambda_{a} \f$
	 * @param tipLossFactor \f$ B_{T} \f$
	 * @param epsilon0 \f$ \epsilon_{0} \f$
	 * @param swashplateAngles \f$ \left[\begin{array}{c}
				\theta_{collective}\\
				\theta_{twist}\\
				\theta_{swash_{side}}\\
				\theta_{swash_{front}}\end{array}\right]\f$
	 * @param flappingAngles \f$ \left[\begin{array}{c}
				\theta_{flap_{cone}}\\
				\theta_{flap_{front}}\\
				\theta_{flap_{side}}\end{array}\right]\f$
	 * @param normalizedAerodynamicPitchVelocity \f$ \overline{q} \f$
     * @return \f$\overline{f_{a_{y}|IERA}^{\curvearrowright}}=-\frac{\sigma\cdot a_{0}}{2}\left(\begin{array}{c}
					-\left(\frac{B_{T}^{3}\mu}{3}\right)\theta_{flap_{cone}}+\left(\frac{B_{T}^{4}\mu}{8}\right)\overline{q_{a}}\ldots\\
					\ldots+\left(\frac{B_{T}^{4}}{8}+\frac{B_{T}^{2}\mu^{2}}{16}\right)\theta_{swash_{side}}+\left(\frac{B_{T}^{4}}{8}+\frac{B_{T}^{2}\mu^{2}}{16}\right)\theta_{flap_{side}}\end{array}\right)
					\f$
     */
    double computeYMomentFactor(double advanceRatio, double aerodynamicInflowRatio, double tipLossFactor, double epsilon0, vector<double> swashplateAngles, vector<double> flappingAngles, double normalizedAerodynamicPitchVelocity) const;

    /*!
     *
	 * @param advanceRatio \f$ \mu \f$
	 * @param aerodynamicInflowRatio \f$ \lambda_{a} \f$
	 * @param tipLossFactor \f$ B_{T} \f$
	 * @param epsilon0 \f$ \epsilon_{0} \f$
	 * @param swashplateAngles \f$ \left[\begin{array}{c}
				\theta_{collective}\\
				\theta_{twist}\\
				\theta_{swash_{side}}\\
				\theta_{swash_{front}}\end{array}\right]\f$
	 * @param flappingAngles \f$ \left[\begin{array}{c}
				\theta_{flap_{cone}}\\
				\theta_{flap_{front}}\\
				\theta_{flap_{side}}\end{array}\right]\f$
     * @return \f$\overline{f_{torque_{flap}}^{\curvearrowright}}=\frac{\sigma\cdot a_{0}}{2}\left(\begin{array}{c}
				\left(\frac{B_{T}^{4}}{4}+\frac{B_{T}^{2}\mu^{2}}{4}\right)\epsilon_{0}\ldots\\
				\ldots-\left(\left(\frac{B_{T}^{3}}{3}\right)\theta_{collective}+\left(\frac{B_{T}^{4}}{4}\right)\theta_{twist}+\left(\frac{B_{T}^{2}}{2}\right)\lambda_{a}\right)\lambda_{a}\\
				\ldots-\left(\frac{B_{T}^{2}\mu^{2}}{4}\right)\theta_{flap_{cone}}^{2}-\left(\frac{B_{T}^{4}}{8}+\frac{3B_{T}^{2}\mu^{2}}{16}\right)\theta_{flap_{front}}^{2}\\
				\ldots-\left(\frac{B_{T}^{4}}{8}+\frac{B_{T}^{2}\mu^{2}}{16}\right)\theta_{flap_{side}}^{2}+\left(\frac{B_{T}^{3}\mu}{3}\right)\theta_{flap_{cone}}\theta_{flap_{side}}-\left(\frac{B_{T}^{2}\mu}{2}\right)\theta_{flap_{front}}\lambda_{a}\end{array}\right)
				\f$
     */
    double computeZFlappingMomentFactor(double advanceRatio, double aerodynamicInflowRatio, double tipLossFactor, double epsilon0, vector<double> swashplateAngles, vector<double> flappingAngles) const;

    /*!
     *
	 * @param advanceRatio \f$ \mu \f$
	 * @param aerodynamicInflowRatio \f$ \lambda_{a} \f$
	 * @param tipLossFactor \f$ B_{T} \f$
	 * @param swashplateAngles \f$ \left[\begin{array}{c}
				\theta_{collective}\\
				\theta_{twist}\\
				\theta_{swash_{side}}\\
				\theta_{swash_{front}}\end{array}\right]\f$
	 * @param flappingAngles \f$ \left[\begin{array}{c}
				\theta_{flap_{cone}}\\
				\theta_{flap_{front}}\\
				\theta_{flap_{side}}\end{array}\right]\f$
     * @return \f$\overline{f_{torque_{swash}}^{\curvearrowright}}=\frac{\sigma\cdot a_{0}}{2}\left(\begin{array}{c}
				\left(\frac{B_{T}^{4}}{8}-\frac{B_{T}^{2}\mu^{2}}{16}\right)\theta_{swash_{front}}\theta_{flap_{front}}\ldots\\
				\ldots-\left(\frac{B_{T}^{2}\mu}{4}\right)\theta_{swash_{front}}\lambda_{a}+\left(\frac{B_{T}^{3}\mu}{6}\right)\theta_{swash_{side}}\theta_{flap_{cone}}\\
				\ldots-\left(\frac{B_{T}^{4}}{8}+\frac{3B_{T}^{2}\mu^{2}}{16}\right)\theta_{swash_{side}}\theta_{flap_{side}}\end{array}\right)
				\f$
     */
    double computeZSwashplateMomentFactor(double advanceRatio, double aerodynamicInflowRatio, double tipLossFactor, vector<double> swashplateAngles, vector<double> flappingAngles) const;

    /*!
     *
	 * @param advanceRatio \f$ \mu \f$
	 * @param aerodynamicInflowRatio \f$ \lambda_{a} \f$
	 * @param tipLossFactor \f$ B_{T} \f$
	 * @param swashplateAngles \f$ \left[\begin{array}{c}
				\theta_{collective}\\
				\theta_{twist}\\
				\theta_{swash_{side}}\\
				\theta_{swash_{front}}\end{array}\right]\f$
	 * @param flappingAngles \f$ \left[\begin{array}{c}
				\theta_{flap_{cone}}\\
				\theta_{flap_{front}}\\
				\theta_{flap_{side}}\end{array}\right]\f$
	 * @param normalizedAerodynamicRollVelocity \f$ \overline{p} \f$
	 * @param normalizedAerodynamicPitchVelocity \f$ \overline{q} \f$
     * @return \f$\overline{f_{torque_{pq}}^{\curvearrowright}}=\frac{\sigma\cdot a_{0}}{2}\left(\begin{array}{c}
				\left(-\frac{B_{T}^{4}}{8}\theta_{swash_{front}}-\frac{B_{T}^{3}\mu}{6}\theta_{collective}-\frac{B_{T}^{4}\mu}{8}\theta_{twist}-\frac{B_{T}^{4}}{8}\overline{p_{a}}+\frac{B_{T}^{4}}{4}\theta_{flap_{front}}\right)\overline{p_{a}}\ldots\\
				\ldots+\left(-\frac{B_{T}^{4}}{8}\theta_{swash_{side}}+\frac{B_{T}^{3}\mu}{3}\theta_{flap_{cone}}-\frac{B_{T}^{4}}{8}\overline{q_{a}}-\frac{B_{T}^{4}}{4}\theta_{flap_{side}}\right)\overline{q_{a}}\end{array}\right)
				\f$
     */
    double computeZRollPitchMomentFactor(double advanceRatio, double aerodynamicInflowRatio, double tipLossFactor, vector<double> swashplateAngles, vector<double> flappingAngles, double normalizedAerodynamicRollVelocity, double normalizedAerodynamicPitchVelocity) const;

    /*!
     *
	 * @param advanceRatio \f$ \mu \f$
	 * @param aerodynamicInflowRatio \f$ \lambda_{a} \f$
	 * @param tipLossFactor \f$ B_{T} \f$
	 * @param epsilon0 \f$ \epsilon_{0} \f$
	 * @param swashplateAngles \f$ \left[\begin{array}{c}
				\theta_{collective}\\
				\theta_{twist}\\
				\theta_{swash_{side}}\\
				\theta_{swash_{front}}\end{array}\right]\f$
	 * @param flappingAngles \f$ \left[\begin{array}{c}
				\theta_{flap_{cone}}\\
				\theta_{flap_{front}}\\
				\theta_{flap_{side}}\end{array}\right]\f$
	 * @param normalizedAerodynamicRollVelocity \f$ \overline{p} \f$
	 * @param normalizedAerodynamicPitchVelocity \f$ \overline{q} \f$
     * @return \f$\overline{\mathbf{f}_{a_{z}|IERA}^{\curvearrowright}}=\overline{f_{torque_{flap}}^{\curvearrowright}}+\overline{f_{torque_{swash}}^{\curvearrowright}}+\overline{f_{torque_{pq}}^{\curvearrowright}}\f$
     */
    double computeZMomentFactor(double advanceRatio, double aerodynamicInflowRatio, double tipLossFactor, double epsilon0, vector<double> swashplateAngles, vector<double> flappingAngles, double normalizedAerodynamicRollVelocity, double normalizedAerodynamicPitchVelocity) const;
    /*!
     * @param swashplateAngles \f$ \left[\begin{array}{c}
				\theta_{collective}\\
				\theta_{twist}\\
				\theta_{swash_{side}}\\
				\theta_{swash_{front}}\end{array}\right]\f$
     * @param advanceRatio \f$ \mu \f$
     * @param inertialInflowRatio \f$ \lambda_{i} \f$
     * @param aerodynamicInflowRatio \f$ \lambda_{a} \f$
     * @return \f$ \lambda_{a}=\frac{w_{a}}{\left\Vert \mathbf{v}_{tan}\right\Vert }=\lambda_{i}-\lambda_{w} \$ The (aerodynamic) inflow ratio, solved
     */
    double solveImplicitThrustWashRelation(double & refTipLossFactor, double & refEpsilon0, double normalizedAerodynamicRollVelocity, vector<double> swashplateAngles, double advanceRatio, double inertialInflowRatio, double aerodynamicInflowRatio) const;


    /*!
     *
	 * @param advanceRatio \f$ \mu \f$
	 * @param aerodynamicInflowRatio \f$ \lambda_{a} \f$
	 * @param tipLossFactor \f$ B_{T} \f$
	 * @param epsilon0 \f$ \epsilon_{0} \f$

	 * @param normalizedNaturalFlappingFrequency \f$ \overline{\omega}_{flap}\f$
     * @param normalizedRotorVelocity \f$ \overline{\Omega}_{rotor}\f$
	 * @param normalizedAerodynamicRollVelocity \f$ \overline{p} \f$
	 * @param normalizedAerodynamicPitchVelocity \f$ \overline{q} \f$
	 * @param swashplateAngles \f$ \left[\begin{array}{c}
				\theta_{collective}\\
				\theta_{twist}\\
				\theta_{swash_{side}}\\
				\theta_{swash_{front}}\end{array}\right]\f$
	 * @param flappingAngles \f$ \left[\begin{array}{c}
				\theta_{flap_{cone}}\\
				\theta_{flap_{front}}\\
				\theta_{flap_{side}}\end{array}\right]\f$
     * @param accelerationVector \f$ \mathbf{a} \f$
     * @return the solution for \f$ \left[\begin{array}{c}
				\theta_{flap_{cone}}\\
				\theta_{flap_{front}}\\
				\theta_{flap_{side}}\end{array}\right]\f$
     */
    vector<double> solveFlappingEquation(double lockNumber, double tipLossFactor, double epsilon0, double advanceRatio, double aerodynamicInflowRatio, double normalizedNaturalFlappingFrequency, double normalizedRotorVelocity, double normalizedAerodynamicRollVelocity, double normalizedAerodynamicPitchVelocity, vector<double> flappingAngles, vector<double> swashplateAngles, vector<double> accelerationVector) const;


    double computeConingAngle(HubType hubtype, double coningAngle, double precone);



	// inherited
	void computeAerodynamicVelocities(double density);

	// inherited
    void computeForcesAndMomentsAtIELA(IndividualElement & centerOfGravity, double density);

    Rotor(int numberOfBlades, double radius, double chord, double linearMass, double delta3Angle, HubType hubType, double preconeAngle, double liftCurveSlope, vector<double> dragCoefficientParameters, double horizontalHubOffset, double verticalHubOffset, double hubSpringConstant, ControlRigging collectivePitchRigging, ControlRigging lateralSwashplateRigging, ControlRigging longitudinalSwashplateRigging, double twistAngle, matrix<double> selfInducedWashVelocityModel);
    ~Rotor();

    /*!
     * Cyclic function (a.k.a. swashplate function)
     * @pre see individual value setters
     * @param collectivePitch \f$ \theta_{collective} \f$
     * @param lateralSwashplateAngle \f$ \theta_{swash_{side}} \f$
     * @param longitudinalSwashplateAngle  \f$ \theta_{swash_{front}} \f$
     * @post stick positions are updated to values within range, as close as possible to the specified values
     */
    void setCyclicAngles(double collectivePitch, double lateralSwashplateAngle, double longitudinalSwashplateAngle);

    /*!
     * Set cyclic (swashplate) angles via stick positions
     * @pre see individual value setters
     * @param collectivePitchStickPosition \f$ \theta_{collective}^{Stick} \f$
     * @param lateralSwashplateAngleStickPosition \f$ \theta_{collective}^{stick} \f$
     * @param longitudinalSwashplateAngleStickPosition \f$ \theta_{swash_{side}}^{stick} \f$
     * @post cyclic angles positions are updated to values within range, as close as possible to the specified values
     */
    void setCyclicStickPosition(double collectivePitchStickPosition, double lateralSwashplateAngleStickPosition, double longitudinalSwashplateAngleStickPosition);

    /*!
     * @pre  \f$ \theta_{collective}^{min} < \theta_{collective} < \theta_{collective}^{max}\f$
     * @param collectivePitch \f$ \theta_{collective} \f$ in radians
     * @post stick position is updated to a value within range, as close as possible to the specified value
     */
    bool setCollectivePitch(double collectivePitch)
    {

    	return this->collectivePitch.setDegreeOfFreedomAngle(collectivePitch);
    }

    /*!
     * @pre \f$ \theta_{collective}^{stick_{min}} < \theta_{collective}^{stick} < \theta_{collective}^{stick_{max}}\f$
     * @param collectivePitchStickPosition \f$ \theta_{collective}^{Stick} \f$
     * @post collectivePitch angle is updated to a value within range, as close as possible to the specified value
     */
    void setCollectivePitchStickPosition(double collectivePitchStickPosition)
    {
    	this->collectivePitch.setStickPosition(collectivePitchStickPosition);
    }

    /*!
     * @return \f$ \theta_{collective} \f$
     */
    double getCollectivePitch()
    {
    	return collectivePitch.getDegreeOfFreedomAngle();
    }

    /*!
     * @return \f$ \theta_{collective}^{stick} \f$
     */
    double getCollectivePitchStickPosition()
    {
    	return collectivePitch.getStickPosition();
    }

    /*!
     * @pre \f$ \theta_{swash_{side}}^{min} < \theta_{swash_{side}} < \theta_{swash_{side}}^{max}\f$
     * @param lateralSwashplateAngle \f$ \theta_{swash_{side}} \f$
     * @post stick position is updated to a value within range, as close as possible to the specified value
     */
    bool setLateralSwashplateAngle(double lateralSwashplateAngle)
    {
    	return this->lateralSwashplateAngle.setDegreeOfFreedomAngle(lateralSwashplateAngle);
    }

    /*!
     * @pre \f$ \theta_{swash_{side}}^{stick_{min}} < \theta_{swash_{side}}^{stick} < \theta_{swash_{side}}^{stick_{max}}\f$
     * @param lateralSwashplateStickPosition \f$ \theta_{swash_{side}}^{stick} \f$
     * @post lateralSwashplateAngle is updated to a value within range, as close as possible to the specified value
     */
    bool setLateralSwashplateStickPosition(double lateralSwashplateStickPosition)
    {
    	return this->lateralSwashplateAngle.setStickPosition(lateralSwashplateStickPosition);
    }

    /*!
     * @return \f$ \theta_{swash_{side}} \f$
     */
    double getLateralSwashplateAngle()
    {
    	return lateralSwashplateAngle.getDegreeOfFreedomAngle();
    }

    /*!
     * @return \f$ \theta_{swash_{side}}^{stick} \f$
     */
    double getLateralSwashplateStickPosition()
    {
    	return lateralSwashplateAngle.getStickPosition();
    }

    /*!
     * @pre \f$ \theta_{swash_{front}}^{min} < \theta_{swash_{front}} < \theta_{swash_{front}}^{max}\f$
     * @param longitudinalSwashplateAngle \f$ \theta_{swash_{front}} \f$
     * @post stick position is updated to a value within range, as close as possible to the specified value
     */
    bool setLongitudinalSwashplateAngle(double longitudinalSwashplateAngle)
    {
    	return this->longitudinalSwashplateAngle.setDegreeOfFreedomAngle(longitudinalSwashplateAngle);
    }

    /*!
     * @pre \f$ \theta_{swash_{front}}^{stick_{min}} < \theta_{swash_{front}}^{stick} < \theta_{swash_{front}}^{stick_{max}}\f$
     * @param longitudinalSwashplateStickPosition \f$ \theta_{swash_{front}}^{stick} \f$
     * @post longitudinalSwashplateAngle is updated to a value within range, as close as possible to the specified value
     */
    bool setLongitudinalSwashplateStickPosition(double longitudinalSwashplateStickPosition)
    {
    	return this->longitudinalSwashplateAngle.setStickPosition(longitudinalSwashplateStickPosition);
    }

    /*!
     * @return \f$ \theta_{swash_{front}} \f$
     */
    double getLongitudinalSwashplateAngle()
    {
    	return longitudinalSwashplateAngle.getDegreeOfFreedomAngle();
    }

    /*!
     * @return \f$ \theta_{swash_{front}}^{stick} \f$
     */
    double getLongitudinalSwashplateStickPosition()
    {
    	return longitudinalSwashplateAngle.getStickPosition();
    }

    /*!
     *
     * @return \f$ \Omega_{rotor} \f$
     */
    double getRotorVelocity() const
    {
        return rotorVelocity;
    }

    /*!
     *
     * @param rotorVelocity \f$ \Omega_{rotor} \f$
     */
    void setRotorVelocity(double rotorSpeed)
    {
        this->rotorVelocity = rotorSpeed;
    }

};

#endif /* ROTOR_H_ */
