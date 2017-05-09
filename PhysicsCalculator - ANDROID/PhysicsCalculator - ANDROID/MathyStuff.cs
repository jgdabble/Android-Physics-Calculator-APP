using System;
using PhysicsCalculator___WINDOWS;

namespace PhysicsMath
{
    public class BasicLinearMotionCalculation
    {
        // This is where the variables needed for this class are declared.
        public double Mass;
        public bool mass; //The boolean is to make sure that the double has a value. If it is True, then it has a value, False means that it has no value.

        public double Accel;
        public bool accel;

        public double Velocity;
        public bool velocity;

        public double InitVelocity;
        public bool initvelocity;

        public double Time;
        public bool time;

        public double DeltaX;
        public bool deltax;

        public double DeltaY;
        public bool deltay;

        public bool NotEnoughInfo;

        public double LaunchAngle;
        public bool launchangle;

        public double SurfaceAngle;
        public bool surfaceangle;

        public double FrictionCoefficient;
        public bool frictcoef;

        public double LinMom;
        public bool linmom;

        public double TotalEnergy;
        public bool totalenergy;

        public double KenEnergy;
        public bool kenenergy;

        public double PotentialEnergy;
        public bool potenergy;

        public double GravPotentialEnergy;
        public bool gravpotenergy;

        public double SpringPotentialEnergy;
        public bool springpotenergy;

        public double SpringDisplacement;
        public bool springdisplacement;

        public double SpringConstant;
        public bool springconstant;

        public bool HorizontalSpring;

        public bool InEquil;

        public double NetForce;
        public bool netforce;

        public double AppliedForce;
        public bool appliedforce;

        public double GravForce;
        public bool gravforce;

        public double FrictionForce;
        public bool frictionforce;

        public double SpringForce;
        public bool springforce;

        public double AppliedForceAngle;
        public bool appliedforceangle;

        public double GravAccel;
        public bool gravaccel;

        public bool horizontalvel;
        public bool verticalvel;

        public bool verticalforce;
        public bool horizontalforce;

        public bool verticalgrav;
        public bool horizontalgrav;

        // This is the method that takes care of simple projectiles with a horizontal launch,
        // driving off a cliff, running off a cliff, knocking something off of a table, etc.
        public void Projectiles()
        {
            if(time == false)
            {
                if(deltay && gravaccel == true)
                {
                    Time = Math.Pow((DeltaY / (.5 * GravAccel)), 2);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(deltay == false)
            {
                if(gravaccel && time == true)
                {
                    DeltaY = (.5 * GravAccel) * Math.Pow(Time, 2);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(deltax == false)
            {
                if(velocity && time == true)
                {
                    DeltaX = (.5 * Velocity) * Time;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(velocity == false)
            {
                if(deltax && time == true)
                {
                    Velocity = DeltaX * Time;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

        }

        // This is the method for projectiles that were launched at a non-zero angle,
        // Shooting a ball from a cannon, throwing something up, or down etc.
        public void AngledLaunchProjectile()
        {
            double VerticalVel = Math.Pow((Math.Sin(LaunchAngle) * Velocity), .5);

            double HorizontalVel = Math.Pow((Math.Cos(LaunchAngle) * Velocity), .5);

            if(deltay == false)
            {
                if(verticalvel && gravaccel && time == true)
                {
                    DeltaY = (-VerticalVel) + ((.5 * GravAccel) * Math.Pow(Time, 2));
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(time == false)
            {
                if(deltay && gravaccel && verticalvel == true)
                {
                    Time = Math.Pow((DeltaY / ((.5 * GravAccel) + VerticalVel)), .5);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(horizontalvel == false)
            {
                if(deltax && time == true)
                {
                    HorizontalVel = 2 * (DeltaX / Time);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }
        
            if(deltax == false)
            {
                if(velocity && time == true)
                {
                    DeltaX = (.5 * Velocity) * Time;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }
        }


        // This method calculates the friction between two objects. Nothing too complex.
        public void Friction()
        {
            // If the surface of the calculation is horizontal, then use this.
            if(SurfaceAngle == 0)
            {
                if(appliedforce == false)
                {
                    if(mass && accel && frictionforce == true)
                    {
                        AppliedForce = (Mass * Accel) + FrictionForce;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }

                if(frictionforce == false)
                {
                    if(mass && accel && appliedforce == true)
                    {
                        FrictionForce = (Mass *Accel) - AppliedForce;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }

                if(frictcoef == false)
                {
                    if(frictionforce && gravforce == true)
                    {
                        FrictionCoefficient = FrictionForce / GravForce;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }
            }
        
            // Otherwise some other calculations involving angles will be required.
            else
            {
                double VerticalForce = Math.Sin(AppliedForceAngle) * AppliedForce;

                double HorizontalForce = Math.Cos(AppliedForceAngle) * AppliedForce;

                if(gravforce == false)
                {
                    if(mass && gravaccel == true)
                    {
                        GravForce = Mass * GravAccel;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }

                double HorizontalGrav = Math.Sin(SurfaceAngle) * GravForce;
                double VerticalGrav = Math.Cos(SurfaceAngle) * GravForce;

                if(frictionforce == false)
                {
                    if(mass && accel && appliedforce == true)
                    {
                        FrictionForce = -(Mass * Accel) - AppliedForce;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }

                if(appliedforce == false)
                {
                    if(mass && accel && frictionforce == true)
                    {
                        AppliedForce = (Mass * Accel) + FrictionForce;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }

                if(accel == false)
                {
                    if(horizontalgrav && appliedforce && frictionforce && mass == true)
                    {
                        Accel = (HorizontalGrav + AppliedForce - FrictionForce) / Mass;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }

                if(appliedforceangle == false)
                {
                    if(mass && accel && horizontalforce == true)
                    {
                        FrictionForce = (Mass * Accel) - HorizontalForce;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }
                   
                if(frictcoef == false)
                {
                    if(frictionforce && verticalgrav == true)
                    {
                        FrictionCoefficient = FrictionForce / VerticalGrav;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }
            } 
        }

        // The method for simple linear momentum and collitions.
        public void LinearMomentum()
        {
            if(linmom == false)
            {
                if(mass && velocity == true)
                {
                    LinMom = Mass * Velocity;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(velocity == false)
            {
                if(linmom && mass == true)
                {
                    Velocity = LinMom / Mass;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(mass == false)
            {
                if(linmom && velocity == true)
                {
                    Mass = LinMom / Velocity;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }
        }

        // Method of energy calculations...
        // Watt are you doing?!
        // I'm ex-static for you to move on and continue reading!
        // Ok, I'll stop now...

        // Do you know where electitians buy their equipment? Ohm Depot!
        // For real this time...
        public void Energy()
        {
            if(kenenergy == false)
            {
                if(mass && velocity == true)
                {
                    KenEnergy = (.5 * Mass) * Math.Pow(Velocity, 2);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(gravpotenergy == false)
            {
                if(mass && gravaccel && deltay == true)
                {
                    GravPotentialEnergy = Mass * GravAccel * DeltaY;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(springpotenergy == false)
            {
                if(springconstant && springdisplacement == true)
                {
                    SpringPotentialEnergy = (.5 * SpringConstant) * Math.Pow(SpringDisplacement, 2);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(totalenergy == false)
            {
                // This Calculates the total system energy.
                if(kenenergy && gravpotenergy == true)
                {
                    TotalEnergy = KenEnergy + GravPotentialEnergy;
                }

                else
                {
                    if(kenenergy && springpotenergy == true)
                    {
                        TotalEnergy = KenEnergy + SpringPotentialEnergy;
                    }

                    else
                    {
                        if(mass && velocity && gravpotenergy == true)
                        {
                            TotalEnergy = ((Mass * .5) * Math.Pow(Velocity, 2)) + GravPotentialEnergy;
                        }

                        else
                        {
                            if(mass && velocity && springpotenergy == true)
                            {
                                TotalEnergy = ((.5 * Mass) * Math.Pow(Velocity, 2)) + SpringPotentialEnergy;
                            }

                            else
                            {
                                if(kenenergy && mass && gravaccel && deltay == true)
                                {
                                    TotalEnergy = KenEnergy + (Mass * GravAccel * DeltaY);
                                }

                                else
                                {
                                    if(kenenergy && springdisplacement && springconstant == true)
                                    {
                                        TotalEnergy = KenEnergy + ((.5 * SpringConstant) * Math.Pow(SpringDisplacement, 2));
                                    }

                                    else
                                    {
                                        if(mass && velocity && gravpotenergy && deltay == true)
                                        {
                                            TotalEnergy = ((.5 * Mass) * Math.Pow(Velocity, 2)) + (Mass * GravAccel * DeltaY);
                                        }

                                        else
                                        {
                                            if(mass && velocity && springconstant && springdisplacement == true)
                                            {
                                                TotalEnergy = ((.5 * Mass) * Math.Pow(Velocity, 2)) + ((.5 * SpringConstant) * Math.Pow(SpringDisplacement, 2));
                                            }

                                            else
                                            {
                                                NotEnoughInfo = true;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // Calculates stuff involving springs...
        public void Springs()
        {
            // If the spring is horizontal, do this.
            if(HorizontalSpring == true)
            {
                if(springforce == false)
                {
                    if(springconstant && springdisplacement == true)
                    {
                        SpringForce = SpringConstant * SpringDisplacement;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }

                if(springconstant == false)
                {
                    if(springforce && springdisplacement == true)
                    {
                        SpringConstant = SpringForce / SpringDisplacement;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }

                if(springdisplacement == false)
                {
                    if(springforce && springdisplacement == true)
                    {
                        SpringDisplacement = SpringForce / SpringConstant;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }
            }

            // Otherwise, it's probably a vertical spring.
            else
            {
                // If the vertical spring is is equilibrium, do this.
                if(InEquil == true)
                {
                    if(springforce == false)
                    {
                        if(gravforce == true)
                        {
                            SpringForce = GravForce;
                        }

                        else
                        {
                            if(mass && gravaccel == true)
                            {
                                SpringForce = Mass * GravAccel;
                            }

                            else
                            {
                                if(springdisplacement && springconstant == true)
                                {
                                    SpringForce = SpringConstant * SpringDisplacement;
                                }

                                else
                                {
                                    NotEnoughInfo = true;
                                }
                            }
                        }
                    }

                    if(gravforce == false)
                    {
                        if(springforce == true)
                        {
                            GravForce = SpringForce;
                        }

                        else
                        {
                            if(mass && gravaccel == true)
                            {
                                GravForce = Mass * GravAccel;
                            }

                            else
                            {
                                if(springdisplacement && springdisplacement == true)
                                {
                                    GravForce = SpringConstant * SpringDisplacement;
                                }

                                else
                                {
                                    NotEnoughInfo = true;
                                }
                            }
                        }
                    }

                    if(springdisplacement == false)
                    {
                        if(springforce && springconstant == true)
                        {
                            SpringDisplacement = SpringForce / SpringConstant;
                        }

                        else
                        {
                            if(gravforce && springconstant == true)
                            {
                                SpringDisplacement = GravForce / SpringConstant;
                            }

                            else
                            {
                                if(mass && gravaccel && springconstant == true)
                                {
                                    SpringDisplacement = (Mass * GravAccel) / SpringConstant;
                                }

                                else
                                {
                                    NotEnoughInfo = true;
                                }
                            }
                        }
                    }

                    if(springconstant == false)
                    {
                        if(springforce && springforce == true)
                        {
                            SpringConstant = SpringForce / SpringDisplacement;
                        }

                        else
                        {
                            if(gravforce && springdisplacement == true)
                            {
                                SpringConstant = GravForce / SpringDisplacement;
                            }

                            else
                            {
                                if(mass && gravaccel && springdisplacement == true)
                                {
                                    SpringConstant = (Mass * GravAccel) / SpringDisplacement;
                                }

                                else
                                {
                                    NotEnoughInfo = true;
                                }
                            }
                        }
                    }

                    if(mass == false)
                    {
                        if(gravforce && gravaccel == true)
                        {
                            Mass = GravForce / GravAccel;
                        }

                        else
                        {
                            if(springforce && gravaccel == true)
                            {
                                Mass = SpringForce / GravAccel;
                            }

                            else
                            {
                                if(springconstant && gravaccel && springdisplacement == true)
                                {
                                    Mass = (SpringConstant * SpringDisplacement) / GravAccel;
                                }

                                else
                                {
                                    NotEnoughInfo = true;
                                }
                            }
                        }
                    }

                    if(gravaccel == false)
                    {
                       if(gravforce && mass == true)
                        {
                            GravAccel = GravForce / Mass;
                        }

                        else
                        {
                            if(springforce && gravaccel == true)
                            {
                                GravAccel = SpringForce / Mass;
                            }

                            else
                            {
                                if(springconstant && gravaccel && springdisplacement == true)
                                {
                                    GravAccel = (SpringConstant * SpringDisplacement) / Mass;
                                }

                                else
                                { 
                                    NotEnoughInfo = true;
                                }
                            }
                        } 
                    }
                }

                // Otherwise it's not is equilibrium, which means it gets more complicated.
                else
                {
                    if(springforce == false)
                    {
                        if(springconstant && springdisplacement == true)
                        {
                            SpringForce = SpringConstant * SpringDisplacement;
                        }

                        else
                        {
                            NotEnoughInfo = true;
                        }
                    }

                    if(springconstant == false)
                    {
                        if(springforce && springdisplacement == true)
                        {
                            SpringConstant = SpringForce / SpringDisplacement;
                        }

                        else
                        {
                            NotEnoughInfo = true;
                        }
                    }

                    if(springdisplacement == false)
                    {
                        if(springforce && springconstant == true)
                        {
                            SpringDisplacement = SpringForce / SpringConstant;
                        }

                        else
                        {
                            NotEnoughInfo = true;
                        }
                    }
                }
            }
        } 
    }

    // The Class for, well, circular motion...
    public class CircularMotionCalculation
    {
        double Mass;
        bool mass;

        double Torgue;
        bool torgue;

        double TanVelocity;
        bool tanvelocity;

        double AngularVelocity;
        bool angularvelocity;

        double Radius;
        bool radius;

        double Accel;
        bool accel;

        double AngularAccel;
        bool angularaccel;

        double Period;
        bool period;

        double CentriAccel;
        bool centriaccel;

        double CentriNetForce;
        bool centrinetforce;

        double MinTopVel;
        bool mintopvel;

        double MinBotVel;
        bool minbotvel;

        double MaxVel;
        bool maxvel;

        double Tension;
        bool tension;

        double GravAccel;
        bool gravaccel;

        double PendulumLength;
        bool pendulumlength;

        double AppliedForce;
        bool appliedforce;

        double Theta;
        bool theta;

        double InertMom;
        bool inertmom;

        double Velocity;
        bool velocity;

        double RotKenEnergy;
        bool rotkenenergy;

        double KenEnergy;
        bool kenenergy;

        bool NotEnoughInfo;

        public void HorizontalCircle()
        {
            // Basic Calculations for horizontal Circular motion.
            if(tanvelocity == false)
            {
                if(radius && period == true)
                {
                    TanVelocity = ((2 * Math.PI) * Radius) / Period;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(radius == false)
            {
                if(tanvelocity && period == true)
                {
                    Radius = (TanVelocity * Period) / (2 * Math.PI);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(period == false)
            {
                if(radius && tanvelocity == true)
                {
                    Period = ((2 * Math.PI) * Radius) / TanVelocity;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(centrinetforce == false)
            {
                if(mass && tanvelocity && radius == true)
                {
                    CentriNetForce = (Mass * Math.Pow(TanVelocity, 2)) / Radius;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }
        }

        public void VerticalCircle()
        {
            // Basic Calculations for vertical circular motion.
            if(tanvelocity == false)
            {
                if(radius && period == true)
                {
                    TanVelocity = ((2 * Math.PI) * Radius) / Period;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(radius == false)
            {
                if(tanvelocity && period == true)
                {
                    Radius = (TanVelocity * Period) / (2 * Math.PI);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(period == false)
            {
                if(radius && tanvelocity == true)
                {
                    Period = ((2 * Math.PI) * Radius) / TanVelocity;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(centrinetforce == false)
            {
                if(mass && tanvelocity && radius == true)
                {
                    CentriNetForce = (Mass * Math.Pow(TanVelocity, 2)) / Radius;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(mintopvel == false)
            {
                if(gravaccel && radius == true)
                {
                    MinTopVel = Math.Pow((GravAccel * Radius), .5);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(minbotvel == false)
            {
                if(mintopvel && gravaccel && radius == true)
                {
                    MinBotVel = MinTopVel + Math.Pow((GravAccel * 2 * Radius), .5);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }
        }

        public void Pendulum()
        {
            // Basic calculations for pendulums.
            if(period == false)
            {
                if(pendulumlength == true && gravaccel == true)
                {
                    Period = (2 * Math.PI) * Math.Pow((PendulumLength / GravAccel), .5);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(pendulumlength == false)
            {
                if(period && gravaccel == true)
                {
                    PendulumLength = Math.Pow(((2 * Math.PI) * Period), 2) * GravAccel;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }
        }

        public void RotationalComponents()
        {
            if(torgue == false)
            {
                if(appliedforce && radius == true)
                {
                    Torgue = (AppliedForce * Radius) * Math.Sin(Theta);
                }

                else
                {
                    if(inertmom && angularaccel == true)
                    {
                        Torgue = InertMom * AngularAccel;
                    }

                    else
                    {
                        if(mass && radius && angularaccel == true)
                        {
                            Torgue = (Mass * Math.Pow(Radius, 2)) * AngularAccel;
                        }

                        else
                        {
                            NotEnoughInfo = true;
                        }
                    }
                }
            }

            // Angular velocity, acceleration, displacement, instance of momentum etc.
            if(velocity == false)
            {
                if(radius && angularvelocity == true)
                {
                    Velocity = Radius * AngularVelocity;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(angularvelocity == false)
            {
                if(velocity && radius == true)
                {
                    AngularVelocity = Velocity / Radius;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(radius == false)
            {
                if(velocity && angularvelocity == true)
                {
                    Radius = Velocity / AngularVelocity;
                }

                else
                {
                    if(accel && angularaccel == true)
                    {
                        Radius = Accel / AngularAccel;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }
            }

            if(accel == false)
            {
                if(radius && angularaccel == true)
                {
                    Accel = Radius * AngularAccel;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(angularaccel == false)
            {
                if(accel && radius == true)
                {
                    AngularAccel = Accel / Radius;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }
        }

        // This calculates Rotational kenetic energy, which is different from linear kenetic energy.
        // not really, they are basically the same in every way execpt for the equation, and even that is almost identical.
        public void RotationalKenEnergy()
        {
            if(rotkenenergy == false)
            {
                if(inertmom && angularvelocity == true)
                {
                    RotKenEnergy = (.5 * InertMom) * Math.Pow(AngularVelocity, 2);
                }

                else
                {
                    if(mass && radius && angularvelocity == true)
                    {
                        RotKenEnergy = (.5 * (Mass * Math.Pow(Radius, 2))) * AngularVelocity;
                    }

                    else
                    {
                        if(velocity && radius && inertmom == true)
                        {
                            RotKenEnergy = (.5 * InertMom) * Math.Pow((Velocity / Radius), 2);
                        }

                        else
                        {
                            if(mass && radius && velocity == true)
                            {
                                RotKenEnergy = (.5 * (Mass * Math.Pow(Radius, 2))) * Math.Pow((Velocity / Radius), 2);
                            }

                            else
                            {
                                if(kenenergy && radius == true)
                                {
                                    RotKenEnergy = KenEnergy / Radius;
                                }

                                else
                                {
                                    NotEnoughInfo = true;
                                }
                            }
                        }
                    }
                }
            }

            if(kenenergy == false)
            {
                if(rotkenenergy && radius == true)
                {
                    KenEnergy = RotKenEnergy * Radius;
                }

                else
                {
                    if (mass && radius && angularvelocity == true)
                    {
                        RotKenEnergy = (.5 * (Mass * Math.Pow(Radius, 2))) * AngularVelocity;
                    }

                    else
                    {
                        if (velocity && radius && inertmom == true)
                        {
                            RotKenEnergy = (.5 * InertMom) * Math.Pow((Velocity / Radius), 2);
                        }

                        else
                        {
                            if (mass && radius && velocity == true)
                            {
                                KenEnergy = ((.5 * (Mass * Math.Pow(Radius, 2))) * Math.Pow((Velocity / Radius), 2)) * Radius;
                            }

                            else
                            {
                                NotEnoughInfo = true;
                            }
                        }
                    }
                }
            }

            if(radius == false)
            {
                if(kenenergy && rotkenenergy == true)
                {
                    Radius = KenEnergy / RotKenEnergy;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }
        }
    }

    public class GravityCalculation
    {
        double GravAccel;
        bool gravaccel;

        double GravForce;
        bool gravforce;

        double SatMass;
        bool satmass;

        double PlanMass;
        bool planmass;

        double OrbitalGravForce;
        bool orbitalgravforce;

        double OrbitalVelocity;
        bool orbitalvelocity;

        double EscapeVelocity;
        bool escapevelocity;

        double UniGravConst = 0.0000000000667408F;
   
        double GravPotentialEnergy;
        bool gravpotenergy;

        double OrbitalPeriod;
        bool orbitalperiod;

        double Radius;
        bool radius;

        bool NotEnoughInfo;

        public void Orbits()
        {
            // Calculations for Orbital problems.
            if(gravpotenergy == false)
            {
                if(satmass && planmass && radius == true)
                {
                    GravPotentialEnergy = -(UniGravConst * SatMass * PlanMass) / Radius;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(satmass == false)
            {
                if(gravpotenergy && radius && planmass == true)
                {
                    SatMass = (GravPotentialEnergy * Radius) / -(UniGravConst * PlanMass);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(planmass == false)
            {
                if(gravpotenergy && radius && satmass == true)
                {
                    PlanMass = (GravPotentialEnergy * Radius) / -(UniGravConst * SatMass);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(radius == false)
            {
                if(satmass && planmass && gravpotenergy == true)
                {
                    Radius = -(UniGravConst * SatMass * PlanMass) / GravPotentialEnergy;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(orbitalvelocity == false)
            {
                if(planmass && radius == true)
                {
                    OrbitalVelocity = Math.Pow(((UniGravConst * PlanMass) / Radius), .5);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(orbitalperiod == false)
            {
                if(radius && satmass == true)
                {
                    OrbitalPeriod = (2 * Math.PI) * Math.Pow((Math.Pow(Radius, 3) / (UniGravConst * SatMass)), .5);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }
        }

        public void GravCal()
        {
            // Everything else involving gravity.
            if(gravpotenergy == false)
            {
                if(satmass && planmass && radius == true)
                {
                    GravPotentialEnergy = -(UniGravConst * SatMass * PlanMass) / Radius;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(satmass == false)
            {
                if(gravpotenergy && radius && planmass == true)
                {
                    SatMass = (GravPotentialEnergy * Radius) / -(UniGravConst * PlanMass);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(planmass == false)
            {
                if(gravpotenergy && radius && satmass == true)
                {
                    PlanMass = (GravPotentialEnergy * Radius) / -(UniGravConst * SatMass);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(radius == false)
            {
                if(satmass && planmass && gravpotenergy == true)
                {
                    Radius = -(UniGravConst * SatMass * PlanMass) / GravPotentialEnergy;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(gravaccel == false)
            {
                if(gravforce && satmass == true)
                {
                    GravAccel = GravForce / SatMass;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(gravforce == false)
            {
                if(gravaccel && satmass == true)
                {
                    GravForce = GravAccel * SatMass;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(satmass == false)
            {
                if(gravforce && gravaccel == true)
                {
                    SatMass = GravForce / GravAccel;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }
        }
    }
}

namespace PhysicsSolver
{
    public class BasicCalulation
    {
        // This is where the variables needed for this class are declared.
        public double Mass;
        public bool mass; //The boolean is to make sure that the double has a value. If it is True, then it has a value, False means that it has no value.

        public double Accel;
        public bool accel;

        public double Velocity;
        public bool velocity;

        public double InitVelocity;
        public bool initvelocity;

        public double Time;
        public bool time;

        public double DeltaX;
        public bool deltax;

        public double DeltaY;
        public bool deltay;

        public bool NotEnoughInfo;

        public double LaunchAngle;
        public bool launchangle;

        public double SurfaceAngle;
        public bool surfaceangle;

        public double FrictionCoefficient;
        public bool frictcoef;

        public double LinMom;
        public bool linmom;

        public double TotalEnergy;
        public bool totalenergy;

        public double KenEnergy;
        public bool kenenergy;

        public double PotentialEnergy;
        public bool potenergy;

        public double GravPotentialEnergy;
        public bool gravpotenergy;

        public double SpringPotentialEnergy;
        public bool springpotenergy;

        public double SpringDisplacement;
        public bool springdisplacement;

        public double SpringConstant;
        public bool springconstant;

        public bool HorizontalSpring;

        public bool InEquil;

        public double NetForce;
        public bool netforce;

        public double AppliedForce;
        public bool appliedforce;

        public double GravForce;
        public bool gravforce;

        public double FrictionForce;
        public bool frictionforce;

        public double SpringForce;
        public bool springforce;

        public double AppliedForceAngle;
        public bool appliedforceangle;

        public double GravAccel;
        public bool gravaccel;

        public bool horizontalvel;
        public bool verticalvel;

        public bool verticalforce;
        public bool horizontalforce;

        public bool verticalgrav;
        public bool horizontalgrav;

        public double NormalForce;
        public bool normalforce;

        public void SolveMass()
        {
            double VerticalForce = Math.Sin(AppliedForceAngle) * AppliedForce;
            double HorizontalForce = Math.Cos(AppliedForceAngle) * AppliedForce;

            double HorizontalGrav = Math.Sin(SurfaceAngle) * GravForce;
            double VerticalGrav = Math.Cos(SurfaceAngle) * GravForce;

            if (horizontalgrav && appliedforce && frictionforce && accel == true)
            {
                Mass = (HorizontalGrav + AppliedForce - FrictionForce) / Accel;
            }

            else
            {
                if (velocity && linmom == true)
                {
                    Mass = LinMom / Velocity;
                }

                else
                {
                    if (linmom && velocity == true)
                    {
                        Mass = LinMom / Velocity;
                    }

                    else
                    {
                        if (gravaccel && gravforce == true)
                        {
                            GravAccel = GravForce / Mass;
                        }

                        else
                        {
                            NotEnoughInfo = true;
                        }
                    }
                }
            }
        }

        public void SolveVelocity()
        {
            if (deltax && time == true)
            {
                Velocity = DeltaX * Time;
            }

            else
            {
                if (linmom && mass == true)
                {
                    Velocity = LinMom / Mass;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }
        }

        public void SolveInitVel()
        {
            if (velocity && time && accel == true)
            {
                InitVelocity = Velocity - (Time * Accel);
            }
        }

        public void SolveAccel()
        {
            double VerticalForce = Math.Sin(AppliedForceAngle) * AppliedForce;
            double HorizontalForce = Math.Cos(AppliedForceAngle) * AppliedForce;

            double HorizontalGrav = Math.Sin(SurfaceAngle) * GravForce;
            double VerticalGrav = Math.Cos(SurfaceAngle) * GravForce;

            if (velocity && initvelocity && time == true)
            {
                Accel = (Velocity - InitVelocity) / Time;
            }

            else
            {
                if (horizontalgrav && appliedforce && frictionforce && mass == true)
                {
                    Accel = (HorizontalGrav + AppliedForce - FrictionForce) / Mass;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }
        }

        public void SolveDistance()
        {
            if (velocity && time == true)
            {
                DeltaX = (.5 * Velocity) * Time;
            }

            else
            {
                NotEnoughInfo = true;
            }
        }

        public void SolveHeight()
        {
            if (gravaccel && time == true)
            {
                DeltaY = (.5 * GravAccel) * Math.Pow(Time, 2);
            }

            else
            {
                NotEnoughInfo = true;
            }
        }

        public void SolveGravAccel()
        {
            if (gravforce && mass == true)
            {
                GravAccel = GravForce / Mass;
            }

            else
            {
                if (springforce && gravaccel == true)
                {
                    GravAccel = SpringForce / Mass;
                }

                else
                {
                    if (springconstant && gravaccel && springdisplacement == true)
                    {
                        GravAccel = (SpringConstant * SpringDisplacement) / Mass;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }
            }
        }

        public void SolveAppliedForce()
        {
            if (mass && accel && frictionforce == true)
            {
                AppliedForce = (Mass * Accel) + FrictionForce;
            }

            else
            {
                NotEnoughInfo = true;
            }
        }

        public void SolveAppliedForceAngle()
        {
            double VerticalForce = Math.Sin(AppliedForceAngle) * AppliedForce;
            double HorizontalForce = Math.Cos(AppliedForceAngle) * AppliedForce;

            double HorizontalGrav = Math.Sin(SurfaceAngle) * GravForce;
            double VerticalGrav = Math.Cos(SurfaceAngle) * GravForce;

            if (mass && accel && horizontalforce == true)
            {
                FrictionForce = (Mass * Accel) - HorizontalForce;
            }

            else
            {
                NotEnoughInfo = true;
            }
        }

        public void SolveFrictionForce()
        {
            if (mass && accel && appliedforce == true)
            {
                FrictionForce = (Mass * Accel) - AppliedForce;
            }

            else
            {
                NotEnoughInfo = true;
            }
        }

        public void SolveFrictCoef()
        {
            if (frictionforce && gravforce == true)
            {
                FrictionCoefficient = FrictionForce / GravForce;
            }

            else
            {
                NotEnoughInfo = true;
            }
        }

        public void SolveGravForce()
        {
            if (mass && gravaccel == true)
            {
                GravForce = Mass * GravAccel;
            }

            else
            {
                NotEnoughInfo = true;
            }
        }

        public void SolveNormalForce()
        {
            // will put stuff here later...
        }

        public void SolveTotalEnergy()
        {
            if (kenenergy && gravpotenergy == true)
            {
                TotalEnergy = KenEnergy + GravPotentialEnergy;
            }

            else
            {
                if (kenenergy && springpotenergy == true)
                {
                    TotalEnergy = KenEnergy + SpringPotentialEnergy;
                }

                else
                {
                    if (mass && velocity && gravpotenergy == true)
                    {
                        TotalEnergy = ((Mass * .5) * Math.Pow(Velocity, 2)) + GravPotentialEnergy;
                    }

                    else
                    {
                        if (mass && velocity && springpotenergy == true)
                        {
                            TotalEnergy = ((.5 * Mass) * Math.Pow(Velocity, 2)) + SpringPotentialEnergy;
                        }

                        else
                        {
                            if (kenenergy && mass && gravaccel && deltay == true)
                            {
                                TotalEnergy = KenEnergy + (Mass * GravAccel * DeltaY);
                            }

                            else
                            {
                                if (kenenergy && springdisplacement && springconstant == true)
                                {
                                    TotalEnergy = KenEnergy + ((.5 * SpringConstant) * Math.Pow(SpringDisplacement, 2));
                                }

                                else
                                {
                                    if (mass && velocity && gravpotenergy && deltay == true)
                                    {
                                        TotalEnergy = ((.5 * Mass) * Math.Pow(Velocity, 2)) + (Mass * GravAccel * DeltaY);
                                    }

                                    else
                                    {
                                        if (mass && velocity && springconstant && springdisplacement == true)
                                        {
                                            TotalEnergy = ((.5 * Mass) * Math.Pow(Velocity, 2)) + ((.5 * SpringConstant) * Math.Pow(SpringDisplacement, 2));
                                        }

                                        else
                                        {
                                            NotEnoughInfo = true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        public void SolveKenEnergy()
        {
            if (mass && velocity == true)
            {
                KenEnergy = (.5 * Mass) * Math.Pow(Velocity, 2);
            }

            else
            {
                NotEnoughInfo = true;
            }
        }

        public void SolvePotEnergy()
        {
            if (mass && gravaccel && deltay == true)
            {
                GravPotentialEnergy = Mass * GravAccel * DeltaY;
            }

            else
            {
                NotEnoughInfo = true;
            }
        }

        public void SolveGravPotEnergy()
        {
            if (mass && gravaccel && deltay == true)
            {
                GravPotentialEnergy = Mass * GravAccel * DeltaY;
            }

            else
            {
                NotEnoughInfo = true;
            }
        }

        public void SolveSpringPotEnergy()
        {
            if (springconstant && springdisplacement == true)
            {
                SpringPotentialEnergy = (.5 * SpringConstant) * Math.Pow(SpringDisplacement, 2);
            }

            else
            {
                NotEnoughInfo = true;
            }
        }

        public void SolveSpringForce()
        {
            if (HorizontalSpring == true)
            {
                if (springforce == false)
                {
                    if (springconstant && springdisplacement == true)
                    {
                        SpringForce = SpringConstant * SpringDisplacement;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }
            }

            else
            {
                if (InEquil == true)
                {
                    if (springforce == false)
                    {
                        if (gravforce == true)
                        {
                            SpringForce = GravForce;
                        }

                        else
                        {
                            if (mass && gravaccel == true)
                            {
                                SpringForce = Mass * GravAccel;
                            }

                            else
                            {
                                if (springdisplacement && springconstant == true)
                                {
                                    SpringForce = SpringConstant * SpringDisplacement;
                                }

                                else
                                {
                                    NotEnoughInfo = true;
                                }
                            }
                        }
                    }
                }
            }
        }

        public void SolveSpringConstant()
        {
            if (HorizontalSpring == true)
            {
                if (springconstant == false)
                {
                    if (springforce && springdisplacement == true)
                    {
                        SpringConstant = SpringForce / SpringDisplacement;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }
            }

            else
            {
                if (InEquil == true)
                {
                    if (springconstant == false)
                    {
                        if (springforce && springforce == true)
                        {
                            SpringConstant = SpringForce / SpringDisplacement;
                        }

                        else
                        {
                            if (gravforce && springdisplacement == true)
                            {
                                SpringConstant = GravForce / SpringDisplacement;
                            }

                            else
                            {
                                if (mass && gravaccel && springdisplacement == true)
                                {
                                    SpringConstant = (Mass * GravAccel) / SpringDisplacement;
                                }

                                else
                                {
                                    NotEnoughInfo = true;
                                }
                            }
                        }
                    }
                }

                else
                {
                    if (springforce && springdisplacement == true)
                    {
                        SpringConstant = SpringForce / SpringDisplacement;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }
            }
        }

        public void SolveSpringDisplacement()
        {
            if (HorizontalSpring == true)
            {
                if (springforce && springdisplacement == true)
                {
                    SpringDisplacement = SpringForce / SpringConstant;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            else
            {
                if (InEquil == true)
                {
                    if (springforce && springconstant == true)
                    {
                        SpringDisplacement = SpringForce / SpringConstant;
                    }

                    else
                    {
                        if (gravforce && springconstant == true)
                        {
                            SpringDisplacement = GravForce / SpringConstant;
                        }

                        else
                        {
                            if (mass && gravaccel && springconstant == true)
                            {
                                SpringDisplacement = (Mass * GravAccel) / SpringConstant;
                            }

                            else
                            {
                                NotEnoughInfo = true;
                            }
                        }
                    }
                }
            }
        }

        public void SolveTime()
        {
            double VerticalVel = Math.Pow((Math.Sin(LaunchAngle) * Velocity), .5);
            double HorizontalVel = Math.Pow((Math.Cos(LaunchAngle) * Velocity), .5);

            if (deltay && gravaccel == true)
            {
                Time = Math.Pow((DeltaY / (.5 * GravAccel)), 2);
            }

            else
            {
                if (deltay && gravaccel && verticalvel == true)
                {
                    Time = Math.Pow((DeltaY / ((.5 * GravAccel) + VerticalVel)), .5);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }
        }

        public void SolveAllBasic()
        {
            SolveMass();
            SolveVelocity();
            SolveInitVel();
            SolveAccel();
            SolveHeight();
            SolveDistance();
            SolveGravAccel();
            SolveGravForce();
            SolveTotalEnergy();
            SolveKenEnergy();
            SolvePotEnergy();
            SolveGravPotEnergy();
            SolveSpringPotEnergy();
            SolveSpringForce();
            SolveSpringConstant();
            SolveSpringDisplacement();
            SolveNormalForce();
            SolveTime();
        }
    }

    public class CircularCalculation
    {
        double Mass;
        bool mass;

        double Torgue;
        bool torgue;

        double TanVelocity;
        bool tanvelocity;

        double AngularVelocity;
        bool angularvelocity;

        double Radius;
        bool radius;

        double Accel;
        bool accel;

        double AngularAccel;
        bool angularaccel;

        double Period;
        bool period;

        double CentriAccel;
        bool centriaccel;

        double CentriNetForce;
        bool centrinetforce;

        double MinTopVel;
        bool mintopvel;

        double MinBotVel;
        bool minbotvel;

        double MaxVel;
        bool maxvel;

        double Tension;
        bool tension;

        double GravAccel;
        bool gravaccel;

        double PendulumLength;
        bool pendulumlength;

        double AppliedForce;
        bool appliedforce;

        double Theta;
        bool theta;

        double InertMom;
        bool inertmom;

        double Velocity;
        bool velocity;

        double RotKenEnergy;
        bool rotkenenergy;

        double KenEnergy;
        bool kenenergy;

        bool NotEnoughInfo;

        public void SolveMass()
        {
            // I need to put stuff in here...
        }

        public void SolveTorgue()
        {
            if (appliedforce && radius == true)
            {
                Torgue = (AppliedForce * Radius) * Math.Sin(Theta);
            }

            else
            {
                if (inertmom && angularaccel == true)
                {
                    Torgue = InertMom * AngularAccel;
                }

                else
                {
                    if (mass && radius && angularaccel == true)
                    {
                        Torgue = (Mass * Math.Pow(Radius, 2)) * AngularAccel;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }
            }
        }

        public void SolveTanVel()
        {
            if (radius && period == true)
            {
                TanVelocity = ((2 * Math.PI) * Radius) / Period;
            }

            else
            {
                NotEnoughInfo = true;
            }
        }

        public void SolveAngVel()
        {
            if (velocity && radius == true)
            {
                AngularVelocity = Velocity / Radius;
            }

            else
            {
                NotEnoughInfo = true;
            }
        }

        public void SolveRadius()
        {
            if (velocity && angularvelocity == true)
            {
                Radius = Velocity / AngularVelocity;
            }

            else
            {
                if (accel && angularaccel == true)
                {
                    Radius = Accel / AngularAccel;
                }
                // some more stuff can be added here to make it more thourough, but this should work for now.
                else
                {
                    NotEnoughInfo = true;
                }
            }
        }

        public void SolveAccel()
        {
            if (radius && angularaccel == true)
            {
                Accel = Radius * AngularAccel;
            }

            else
            {
                NotEnoughInfo = true;
            }
        }

        public void SolveAngAccel()
        {
            if (accel && radius == true)
            {
                AngularAccel = Accel / Radius;
            }

            else
            {
                NotEnoughInfo = true;
            }
        }

        public void SolvePeriod()
        {
            if (radius && tanvelocity == true)
            {
                Period = ((2 * Math.PI) * Radius) / TanVelocity;
            }

            else
            {
                if (pendulumlength == true && gravaccel == true)
                {
                    Period = (2 * Math.PI) * Math.Pow((PendulumLength / GravAccel), .5);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }
        }

        public void SolveCentriAccel()
        {
            // I will finish, and start, this method later.
        }

        public void SolveCentriNetForce()
        {
            if (mass && tanvelocity && radius == true)
            {
                CentriNetForce = (Mass * Math.Pow(TanVelocity, 2)) / Radius;
            }

            else
            {
                NotEnoughInfo = true;
            }
        }

        public void SolveMinTopVel()
        {
            if (gravaccel && radius == true)
            {
                MinTopVel = Math.Pow((GravAccel * Radius), .5);
            }

            else
            {
                NotEnoughInfo = true;
            }
        }

        public void SolveMinBotVel()
        {
            if (mintopvel && gravaccel && radius == true)
            {
                MinBotVel = MinTopVel + Math.Pow((GravAccel * 2 * Radius), .5);
            }

            else
            {
                NotEnoughInfo = true;
            }
        }

        public void SolveMaxVel()
        {
            // I'll work on this later...
        }

        public void SolveTension()
        {
            // finish this later...
        }

        public void SolveGravAccel()
        {
            if (pendulumlength == true && period == true)
            {
                GravAccel = (Math.Pow((2 * Math.PI), 2) * PendulumLength) / Math.Pow(Period, 2);
            }

            else
            {
                NotEnoughInfo = true;
            }
        }

        public void SolvePendulumLenth()
        {
            if (period && gravaccel == true)
            {
                PendulumLength = Math.Pow(((2 * Math.PI) * Period), 2) * GravAccel;
            }

            else
            {
                NotEnoughInfo = true;
            }
        }

        public void SolveAppliedForce()
        {
            // put stuff here.
        }
    }
}
