/*
 * HalfWing.h
 *
 *  Created on: 1-Aug-2008
 *      Author: malem303
 */

#ifndef HALFWING_H_
#define HALFWING_H_

#include "IndividualElement.h"

enum HalfWingType {LEFT_WING = -1, RIGHT_WING = 1, FIN = 1};

class HalfWing: public IndividualElement
{
private:

    HalfWingType halfWingType;


	/*!  a.k.a \f$ l_{fin} \f$ */
    double halfWingSpan;

    /*! \f$ c_{root} \f$ */
    double chordAtRoot;

    /*! \f$ c_{tip} \f$ */
    double chordAtTip;

    /*! \f$ (\pm\Gamma,\theta,\pm\Lambda) \f$ used to build  \f$ \mathbf{T}_{WRA-IERA}\f$*/
    vector<double> rotationAnglesWRAFromIERA;

	/*! \f$ \alpha_{0} \f$, also known as \f$ \dot{C_{l}(\alpha)}_{2D} \f$ */
	double liftCurveSlope;
    /*! The function used in Dreier, 2007, p456 is \f$ C_{d}\left(\alpha\right)=c_{d_{0}}+\alpha\left(c_{d_{1}}+\alpha\cdot c_{d_{2}}\right) \f$ */
    vector<double> dragCoefficientParameters;

    /*! The function used in Dreier, 2007, p456 is \f$ C_{m}\left(\alpha\right)=c_{m_{0}}+c_{m_{\alpha}}\cdot\alpha \f$ */
    vector<double> momentCoefficientParameters;


    /*!
     *
     * @return \f$ \lambda=\frac{c_{tip}}{c_{root}} \f$
     */
    double computeTaperRatio() const;

    /*!
     *
     * @return \f$ \overline{c} \f$
     */
    double computeMeanAerodynamicChord() const;

    /*!
     *
     * @return \f$ \frac{l_{wing}\left(2\lambda+1\right)}{6\left(\lambda+1\right)} \f$
     */
    double computeCentroid() const;

    /*!
     *
     * @return \f$ a_{r} = \frac{l_{wing}^{2}}{area_{wing}}\f$
     */
    double computeAspectRatio() const;


    /*!
     *
     * @param meanAerodynamicChordVelocityWRA \f$ \mathbf{v}_{\overline{c}_{right}|WRA_{right}} \f$
     * @return \f$ \alpha=\tan^{-1}\left(\frac{w_{\overline{c}_{right}|WRA_{right}}}{u_{\overline{c}_{right}|WRA_{right}}}\right) \f$
     */
    double computeAngleOfAttack(vector<double> meanAerodynamicChordVelocityWRA) const;

    /*!
     *
     * @param angleOfAttack \f$ \alpha \f$
     * @return \f$ \dot{C_{l}\left(\alpha\right)}_{3D}=\dot{C_{l}(\alpha)}_{2D}\cdot\frac{a_{r}}{a_{r}+2\frac{\left(a_{r}+4\right)}{\left(a_{r}+2\right)}}\cdot\alpha\f$
     */
    double computeLiftCoefficient(double angleOfAttack) const;
    double computeDragCoefficient(double angleOfAttack) const;
    double computeMomentCoefficient(double angleOfAttack) const;

    /*!
     * The Halfwing's area
     * @note the Halfwing's span is half the wing's span. The fin's span is the full halfwing's span
     * @return \f$ area_{fin}=\int_{0}^{l_{fin}}c(y)dy=\frac{l_{fin}\cdot c_{root}\cdot\left(1+\lambda\right)}{2} \f$
     */
    double computeHalfWingArea() const;
    double computeSideSlipAngle(vector<double> meanAerodynamicChordVelocityWRA) const;
public:
	HalfWing(HalfWingType halfWingType, double halfWingSpan, double chordAtRoot, double chordAtTip, vector<double> rotationAnglesWRAFromIERA, double liftCurveSlope,  vector<double> dragCoefficientParameters, vector<double> momentCoefficientParameters, matrix<double> selfInducedWashVelocityModel, vector<double> distanceIELAFromReferenceAxes = Constants::ZERO_VEC_3, vector<double> rotationAnglesIERAFromIELA = Constants::ZERO_VEC_3);
	virtual ~HalfWing();

	// inherited doc
	void computeForcesAndMomentsAtIELA(IndividualElement & centerOfGravity, double density);
};

#endif /* HALFWING_H_ */
