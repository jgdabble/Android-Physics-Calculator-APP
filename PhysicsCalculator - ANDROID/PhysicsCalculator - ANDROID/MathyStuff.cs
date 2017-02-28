using System;

namespace PhysicsMath
{
    public class BasicLinearMotion
    {
        // This is where almost all the variables are declared, there are alot...
        // They are somewhat seperated by category.
        double Mass;
        double Accel;
        double Velocity;
        double initvelocity;
        double Time;
        double DeltaX;
        double DeltaY;
        bool NotEnoughInfo;
        double LaunchAngle;
        double SurfaceAngle;
        double FrictionCoefficient;
        double LinMom;

        double TotalEnergy;
        double KenEnergy;
        double PotentialEnergy;
        double GravPotentialEnergy;
        double SpringPotentialEnergy;

        double SpringDisplacement;
        double SpringConstant;
        bool HorizontalSpring;
        bool InEquil;

        double NetForce;
        double AppliedForce;
        double GravForce;
        double FrictionForce;
        double SpringForce;
        double AppliedForceAngle;

        double GravAccel;

        // This is the method that takes care of simple projectiles with a horizontal launch,
        // driving off a cliff, running off a cliff, knocking something off of a table, etc.
        public void Projectiles()
        {
            if(Time.HasValue == false)
            {
                if(DeltaY.HasValue && GravAccel.HasValue == true)
                {
                    Time = System.Math.Pow((DeltaY / (.5 * GravAccel)), 2);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(DeltaY.HasValue == false)
            {
                if(GravAccel.HasValue && Time.HasValue == true)
                {
                    DeltaY = (.5 * GravAccel) * Math.Pow(Time, 2);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(DeltaX.HasValue == false)
            {
                if(Velocity.HasValue && Time.HasValue == true)
                {
                    DeltaX = (.5 * Velocity) * Time;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(Velocity.HasValue == false)
            {
                if(DeltaX.HasValue && Time.HasValue == true)
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
            double verticalvel = Math.Pow((Math.Sin(LaunchAngle) * Velocity), .5);

            double horizontalvel = Math.Pow((Math.Cos(LaunchAngle) * Velocity), .5);

            if(DeltaY.HasValue == false)
            {
                if(verticalvel.HasValue && GravAccel.HasValue && Time.HasValue == true)
                {
                    DeltaY = (-verticalvel) + ((.5 * GravAccel) * Math.Pow(Time, 2));
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(Time.HasValue == false)
            {
                if(DeltaY.HasValue && GravAccel.HasValue && verticalvel.HasValue == true)
                {
                    Time = Math.Pow((DeltaY / ((.5 * GravAccel) + verticalvel)), .5);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(horizontalvel.HasValue == false)
            {
                if(DeltaX.HasValue && Time.HasValue == true)
                {
                    horizontalvel = 2 * (DeltaX / Time);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }
        
            if(DeltaX.HasValue == false)
            {
                if(Velocity.HasValue && Time.HasValue == true)
                {
                    DeltaX = (.5 * Velocity) * Time;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }
        }


        // This method calculates the friction between two objects. Pretty simple really.
        public void Friction()
        {
            // If the surface of the calculation is horizontal, then use this.
            if(SurfaceAngle == 0)
            {
                if(AppliedForce.HasValue == false)
                {
                    if(Mass.HasValue && Accel.HasValue && FrictionForce.HasValue == true)
                    {
                        AppliedForce = (Mass * Accel) + FrictionForce;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }

                if(FrictionForce.HasValue == false)
                {
                    if(Mass.HasValue && Accel.HasValue && AppliedForce.HasValue == true)
                    {
                        FrictionForce = (Mass *Accel) - AppliedForce;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }

                if(FrictionCoefficient.HasValue == false)
                {
                    if(FrictionForce.HasValue && GravForce.HasValue == true)
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
                double verticalforce = Math.Sin(AppliedForceAngle) * AppliedForce;

                double horizontalforce = Math.Cos(AppliedForceAngle) * AppliedForce;

                bool gravcompsfound = true;

                if(GravForce.HasValue == false)
                {
                    if(Mass.HasValue && GravAccel.HasValue == true)
                    {
                        GravForce = Mass * GravAccel;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }

                double horizontalgrav = Math.Sin(SurfaceAngle) * GravForce;
                double verticalgrav = Math.Cos(SurfaceAngle) * GravForce;

                if(FrictionForce.HasValue = false)
                {
                    if(Mass.HasValue && Accel.HasValue && AppliedForce.HasValue == false)
                    {
                        FrictionForce = -(Mass * Accel) - AppliedForce;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }

                if(AppliedForce.HasValue == false)
                {
                    if(Mass.HasValue && Accel.HasValue && FrictionForce.HasValue == true)
                    {
                        AppliedForce = (Mass * Accel) + FrictionForce;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }

                if(Accel.HasValue == false)
                {
                    if(horizontalgrav.HasValue && AppliedForce.HasValue && FrictionForce.HasValue && Mass.HasValue == true)
                    {
                        Accel = (horizontalgrav + AppliedForce - FrictionForce) / Mass;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }

                if(AppliedForceAngle.HasValue == false)
                {
                    if(Mass.HasValue && Accel.HasValue && horizontalforce.HasValue == true)
                    {
                        FrictionForce = (Mass * Accel) - horizontalforce;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }
                   
                if(FrictionCoefficient.HasValue == false)
                {
                    if(FrictionForce.HasValue && verticalgrav.HasValue == true)
                    {
                        FrictionCoefficient = FrictionForce / verticalgrav;
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
            if(LinMom.HasValue == false)
            {
                if(Mass.HasValue && Velocity.HasValue == true)
                {
                    LinMom = Mass * Velocity;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(Velocity.HasValue == false)
            {
                if(LinMom.HasValue && Mass.HasValue == true)
                {
                    Velocity = LinMom / Mass;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(Mass.HasValue == false)
            {
                if(LinMom.HasValue && Velocity.HasValue == true)
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
            if(KenEnergy.HasValue == false)
            {
                if(Mass.HasValue && Velocity.HasValue == true)
                {
                    KenEnergy = (.5 * Mass) * Math.Pow(Velocity, 2);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(GravPotentialEnergy.HasValue == false)
            {
                if(Mass.HasValue && GravAccel.HasValue && DeltaY.HasValue == true)
                {
                    GravPotentialEnergy = Mass * GravAccel * DeltaY;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(SpringPotentialEnergy.HasValue == false)
            {
                if(SpringConstant.HasValue && SpringDisplacement.HasValue == true)
                {
                    SpringPotentialEnergy = (.5 * SpringConstant) * Math.Pow(SpringDisplacement, 2);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(TotalEnergy.HasValue == false)
            {
                // This Calculates the total system energy.
                if(KenEnergy.HasValue && GravPotentialEnergy.HasValue == true)
                {
                    TotalEnergy = KenEnergy + GravPotentialEnergy;
                }

                else
                {
                    if(KenEnergy.HasValue && SpringPotentialEnergy.HasValue == true)
                    {
                        TotalEnergy = KenEnergy + SpringPotentialEnergy;
                    }

                    else
                    {
                        if(Mass.HasValue && Velocity.HasValue && GravPotentialEnergy.HasValue == true)
                        {
                            TotalEnergy = ((Mass * .5) * Math.Pow(Velocity, 2)) + GravPotentialEnergy;
                        }

                        else
                        {
                            if(Mass.HasValue && Velocity.HasValue && SpringPotentialEnergy.HasValue == true)
                            {
                                TotalEnergy = ((.5 * Mass) * Math.Pow(Velocity, 2)) + SpringPotentialEnergy;
                            }

                            else
                            {
                                if(KenEnergy.HasValue && Mass.HasValue && GravAccel.HasValue && DeltaY.HasValue == true)
                                {
                                    TotalEnergy = KenEnergy + (Mass * GravAccel * DeltaY);
                                }

                                else
                                {
                                    if(KenEnergy.HasValue && SpringDisplacement.HasValue && SpringConstant.HasValue == true)
                                    {
                                        TotalEnergy = KenEnergy + ((.5 * SpringConstant) * Math.Pow(SpringDisplacement, 2));
                                    }

                                    else
                                    {
                                        if(Mass.HasValue && Velocity.HasValue && GravAccel.HasValue && DeltaY.HasValue == true)
                                        {
                                            TotalEnergy = ((.5 * Mass) * Math.Pow(Velocity, 2)) + (Mass * GravAccel * DeltaY);
                                        }

                                        else
                                        {
                                            if(Mass.HasValue && Velocity.HasValue && SpringConstant.HasValue && SpringDisplacement.HasValue == true)
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
                if(SpringForce.HasValue == false)
                {
                    if(SpringConstant.HasValue && SpringDisplacement.HasValue == true)
                    {
                        SpringForce = SpringConstant * SpringDisplacement;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }

                if(SpringConstant.HasValue == false)
                {
                    if(SpringForce.HasValue && SpringDisplacement.HasValue == true)
                    {
                        SpringConstant = SpringForce / SpringDisplacement;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }

                if(SpringDisplacement.HasValue == false)
                {
                    if(SpringForce.HasValue && SpringDisplacement.HasValue == true)
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
                    if(SpringForce.HasValue == false)
                    {
                        if(GravForce.HasValue == true)
                        {
                            SpringForce = GravForce;
                        }

                        else
                        {
                            if(Mass.HasValue && GravAccel.HasValue == true)
                            {
                                SpringForce = Mass * GravAccel;
                            }

                            else
                            {
                                if(SpringDisplacement.HasValue && SpringConstant.HasValue == true)
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

                    if(GravForce.HasValue == false)
                    {
                        if(SpringForce.HasValue == true)
                        {
                            GravForce = SpringForce;
                        }

                        else
                        {
                            if(Mass.HasValue && GravAccel.HasValue == true)
                            {
                                GravForce = Mass * GravAccel;
                            }

                            else
                            {
                                if(SpringDisplacement.HasValue && SpringConstant.HasValue == true)
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

                    if(SpringDisplacement.HasValue == false)
                    {
                        if(SpringForce.HasValue && SpringConstant.HasValue == true)
                        {
                            SpringDisplacement = SpringForce / SpringConstant;
                        }

                        else
                        {
                            if(GravForce.HasValue && SpringConstant.HasValue == true)
                            {
                                SpringDisplacement = GravForce / SpringConstant;
                            }

                            else
                            {
                                if(Mass.HasValue && GravAccel.HasValue && SpringConstant.HasValue == true)
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

                    if(SpringConstant.HasValue == false)
                    {
                        if(SpringForce.HasValue && SpringDisplacement.HasValue == true)
                        {
                            SpringConstant = SpringForce / SpringDisplacement;
                        }

                        else
                        {
                            if(GravForce.HasValue && SpringDisplacement.HasValue == true)
                            {
                                SpringConstant = GravForce / SpringDisplacement;
                            }

                            else
                            {
                                if(Mass.HasValue && GravAccel.HasValue && SpringDisplacement.HasValue == true)
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

                    if(Mass.HasValue == false)
                    {
                        if(GravForce.HasValue && GravAccel.HasValue == true)
                        {
                            Mass = GravForce / GravAccel;
                        }

                        else
                        {
                            if(SpringForce.HasValue && GravAccel.HasValue == true)
                            {
                                Mass = SpringForce / GravAccel;
                            }

                            else
                            {
                                if(SpringConstant.HasValue && GravAccel.HasValue && SpringDisplacement.HasValue == true)
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

                    if(GravAccel.HasValue == false)
                    {
                       if(GravForce.HasValue && GravAccel.HasValue == true)
                        {
                            GravAccel = GravForce / Mass;
                        }

                        else
                        {
                            if(SpringForce.HasValue && GravAccel.HasValue == true)
                            {
                                GravAccel = SpringForce / Mass;
                            }

                            else
                            {
                                if(SpringConstant.HasValue && GravAccel.HasValue && SpringDisplacement.HasValue == true)
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
                    if(SpringForce.HasValue == false)
                    {
                        if(SpringConstant.HasValue && SpringDisplacement.HasValue == true)
                        {
                            SpringForce = SpringConstant * SpringDisplacement;
                        }

                        else
                        {
                            NotEnoughInfo = true;
                        }
                    }

                    if(SpringConstant.HasValue == false)
                    {
                        if(SpringForce.HasValue && SpringDisplacement.HasValue == true)
                        {
                            SpringConstant = SpringForce / SpringDisplacement;
                        }

                        else
                        {
                            NotEnoughInfo = true;
                        }
                    }

                    if(SpringDisplacement.HasValue == false)
                    {
                        if(SpringForce.HasValue && SpringDisplacement.HasValue == true)
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
    public class CircularMotion
    {
        double Mass;
        double Torgue;
        double TanVelocity;
        double AngularVelocity;
        double Radius;
        double Accel;
        double AngularAccel;
        double Period;
        double CentriAccel;
        double CentriNetForce;
        double MinTopVel;
        double MinBotVel;
        double MaxVel;
        double Tension;
        bool NotEnoughInfo;

        public void HorizontalCircle()
        {
            // Basic Calculations for horizontal Circular motion.
            if(TanVelocity.HasValue == false)
            {
                if(Radius.HasValue && Period.HasValue == true)
                {
                    TanVelocity = ((2 * Math.PI) * Radius) / Period;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(Radius.HasValue == false)
            {
                if(TanVelocity.HasValue && Period.HasValue == true)
                {
                    Radius = (TanVelocity * Period) / (2 * Math.PI);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(Period.HasValue == false)
            {
                if(Radius.HasValue && TanVelocity.HasValue == true)
                {
                    Period = ((2 * Math.PI) * Radius) / TanVelocity;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(CentriNetForce.HasValue == false)
            {
                if(Mass.HasValue && TanVelocity.HasValue && Radius.HasValue == true)
                {
                    CentriNetForce = (Mass * Math.Sin(TanVelocity, 2)) / Radius;
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
            if(TanVelocity.HasValue == false)
            {
                if(Radius.HasValue && Period.HasValue == true)
                {
                    TanVelocity = ((2 * Math.PI) * Radius) / Period;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(Radius.HasValue == false)
            {
                if(TanVelocity.HasValue && Period.HasValue == true)
                {
                    Radius = (TanVelocity * Period) / (2 * Math.PI);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(Period.HasValue == false)
            {
                if(Radius.HasValue && TanVelocity.HasValue == true)
                {
                    Period = ((2 * Math.PI) * Radius) / TanVelocity;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(CentriNetForce.HasValue == false)
            {
                if(Mass.HasValue && TanVelocity.HasValue && Radius.HasValue == true)
                {
                    CentriNetForce = (Mass * (TanVelocity ** 2)) / Radius;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(MinTopVel.HasValue == false)
            {
                if(GravAccel.HasValue && Radius.HasValue == true)
                {
                    MinTopVel = (GravAccel * Radius) ** .5;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(MinBotVel.HasValue == false)
            {
                if(MinTopVel.HasValue && GravAccel.HasValue && Radius.HasValue == true)
                {
                    MinBotVel = MinTopVel + (GravAccel * 2 * Radius) ** .5;
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
            if(Period.HasValue == false)
            {
                if(PendulumLength.HasValue == true && GravAccel.HasValue == true)
                {
                    Period = (2 * Math.PI) * ((PendulumLength / GravAccel) ** .5);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(PendulumLength.HasValue == false)
            {
                if(Period.HasValue && GravAccel.HasValue == true)
                {
                    PendulumLength = (((2 * Math.PI) * Period) ** 2) * GravAccel;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }
        }

        public void RotationalComponents()
        {
            if (Torgue.HasValue == false)
            {
                if (AppliedForce.HasValue && Radius.HasValue == true)
                {
                    Torgue = (AppliedForce * Radius) * math.sin(Theta);
                }

                else
                {
                    if (InertMom.HasValue && AngularAccel.HasValue == true)
                    {
                        Torgue = InertMom * AngularAccel;
                    }

                    else
                    {
                        if (Mass.HasValue && Radius.HasValue && AngularAccel.HasValue == true)
                        {
                            Torgue = (Mass * (Radius * *2)) * AngularAccel;
                        }

                        else
                        {
                            NotEnoughInfo = true;
                        }
                    }
                }
            }

            // Angular velocity, acceleration, displacement, instance of momentum etc.
            if(Velocity.HasValue == false)
            {
                if(Radius.HasValue && AngularVelocity.HasValue == true)
                {
                    Velocity = Radius * AngularVelocity;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(AngularVelocity.HasValue == false)
            {
                if(Velocity.HasValue && Radius.HasValue == true)
                {
                    AngularVelocity = Velocity / Radius;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(Radius.HasValue == false)
            {
                if(Velocity.HasValue && AngularVelocity.HasValue == true)
                {
                    Radius = Velocity / AngularVelocity;
                }

                else
                {
                    if(Accel.HasValue && AngularAccel.HasValue == true)
                    {
                        Radius = Accel / AngularAccel;
                    }

                    else
                    {
                        NotEnoughInfo = true;
                    }
                }
            }

            if(Accel.HasValue == false)
            {
                if(Radius.HasValue && AngularAccel.HasValue == true)
                {
                    Accel = Radius * AngularAccel;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(AngularAccel.HasValue == false)
            {
                if(Accel.HasValue && Radius.HasValue == true)
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
        public void RotKenEnergy()
        {
            if(RotKenEnergy.HasValue == false)
            {
                if(InertMom.HasValue && AngularVelocity.HasValue == true)
                {
                    RotKenEnergy = (.5 * InertMom) * (AngularVelocity ** 2);
                }

                else
                {
                    if(Mass.HasValue && Radius.HasValue && AngularVelocity.HasValue == true)
                    {
                        RotKenEnergy = (.5 * (Mass * (Radius ** 2))) * AngularVelocity;
                    }

                    else
                    {
                        if(Velocity.HasValue && Radius.HasValue && InertMom.HasValue == true)
                        {
                            RotKenEnergy = (.5 * InertMom) * ((Velocity / Radius) ** 2);
                        }

                        else
                        {
                            if(Mass.HasValue && Radius.HasValue && Velocity.HasValue == true)
                            {
                                RotKenEnergy = (.5 * (Mass * (Radius ** 2))) * ((Velocity / Radius) ** 2);
                            }

                            else
                            {
                                if(KenEnergy.HasValue && Radius.HasValue == true)
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
        }
    }

    public class Gravity
    {
        double GravAccel;
        double SatMass;
        double PlanMass;
        double OrbitalGravForce;
        double OrvitalVelocity;
        double EscapeVelocity;
        double UniGravConst = 0.0000000000667408F;
        double GravPotentialEnergy;
        double OrbitalPeriod;
        bool NotEnoughInfo;

        public void Orbits()
        {
            // Calculations for Orbital problems.
            if(GravPotentialEnergy.HasValue == false)
            {
                if(UniGravConst.HasValue && SatMass.HasValue && PlanMass.HasValue && Radius.HasValue == true)
                {
                    GravPotentialEnergy = -(UniGravConst * SatMass * PlanMass) / Radius;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(SatMass.HasValue == false)
            {
                if(GravPotentialEnergy.HasValue && Radius.HasValue && PlanMass.HasValue == true)
                {
                    SatMass = (GravPotentialEnergy * Radius) / -(UniGravConst * PlanMass);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(PlanMass.HasValue == false)
            {
                if(GravPotentialEnergy.HasValue && Radius.HasValue && SatMass.HasValue == true)
                {
                    PlanMass = (GravPotentialEnergy * Radius) / -(UniGravConst * SatMass);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(Radius.HasValue == false)
            {
                if(SatMass.HasValue && PlanMass.HasValue && GravPotentialEnergy.HasValue == true)
                {
                    Radius = -(UniGravConst * SatMass * PlanMass) / GravPotentialEnergy;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(OrbitalVelocity.HasValue == false)
            {
                if(PlanMass.HasValue && Radius.HasValue == true)
                {
                    OrbitalVelocity = ((UniGravConst * PlanMass) / Radius) ** .5;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(OrbitalPeriod.HasValue == false)
            {
                if(Radius.HasValue && mass.HasValue == true)
                {
                    OrbitalPeriod = (2 * Math.PI) * ((Radius ** 3) / (UniGravConst * mass)) ** .5;
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
            if(GravPotentialEnergy.HasValue == false)
            {
                if(SatMass.HasValue && PlanMass.HasValue && Radius.HasValue == true)
                {
                    GravPotentialEnergy = -(UniGravConst * SatMass * PlanMass) / Radius;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(SatMass.HasValue == false)
            {
                if(GravPotentialEnergy.HasValue && Radius.HasValue && PlanMass.HasValue == true)
                {
                    SatMass = (GravPotentialEnergy * Radius) / -(UniGravConst * PlanMass);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(PlanMass.HasValue == false)
            {
                if(GravPotentialEnergy.HasValue && Radius.HasValue && SatMass.HasValue == true)
                {
                    PlanMass = (GravPotentialEnergy * Radius) / -(UniGravConst * SatMass);
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(Radius.HasValue == false)
            {
                if(SatMass.HasValue && PlanMass.HasValue && GravPotentialEnergy.HasValue == true)
                {
                    Radius = -(UniGravConst * SatMass * PlanMass) / GravPotentialEnergy;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(GravAccel.HasValue == false)
            {
                if(GravForce.HasValue && mass.HasValue == true)
                {
                    GravAccel = GravForce / Mass;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(GravForce.HasValue == false)
            {
                if(GravAccel.HasValue && Mass.HasValue == true)
                {
                    GravForce = GravAccel * Mass;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }

            if(Mass.HasValue == false)
            {
                if(GravForce.HasValue && GravAccel.HasValue == true)
                {
                    Mass = GravForce / GravAccel;
                }

                else
                {
                    NotEnoughInfo = true;
                }
            }
        }
    }
}
