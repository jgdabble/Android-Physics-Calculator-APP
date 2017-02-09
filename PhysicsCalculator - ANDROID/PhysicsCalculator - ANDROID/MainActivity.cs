using Android.App;
using Android.Widget;
using Android.OS;

namespace PhysicsCalculator___ANDROID
{
    [Activity(Label = "PhysicsCalculator___ANDROID", MainLauncher = true, Icon = "@drawable/icon")]
    public class MainActivity : Activity
    {
        protected override void OnCreate(Bundle bundle)
        {
            base.OnCreate(bundle);
            SetContentView(Resource.Layout.Main);

            bool OnEarth;
            float mass;
            float accel;
            float velocity;
            float initvelocity;

            float TotalEnergy;
            float KenEnergy;
            float PotentialEnergy;
            float GravPotentialEnergy;
            float SpringPotentialEnergy;

            float SpringDisplacement;
            float SpringConstant;
            float amplitude;
            float period;
            float PendulumLength;
            // SOOOOOO MANY FLOATS!!!! THERE'S A ROOTBEER ONE, AND A ORANGE ONE, AND A SPRITE ONE, AND A BOAT!
            float NetForce;
            float Force;
            float GravForce;
            float FrictionForce;
            float SpringForce;

            float GravAccel;
            float SatMass;
            float PlanMass;
            float OrbitalGravForce;
            float OrvitalVelocity;
            float EscapeVelocity;
            float UniGravConst = 0.0000000000667408;
            
            bool OnEarth;
            bool SpringsOn;
            bool PendulumsOn;
            bool CircularMotion;
            bool HasMass;
            bool LinAccel;
            bool UsesEnergy;

            // Is the calculation taking place on earth or some other place in the vast cosmos?
            ToggleButton EarthGrav = FindViewById<ToggleButton>(Resource.Id.EarthGravToggle);
            EarthGrav.Click += (o, e) =>
            {
                if (EarthGrav.Checked)
                    OnEarth = true;

                else
                    OnEarth = false;
            };

            // Is there an active spring working in the problem?
            ToggleButton SpringsToggle = FindViewById<ToggleButton>(Resource.Id.SpringToggle);
            SpringsToggle.Click += (o, e) =>
            {
                if (SpringsToggle.Checked)
                    SpringsOn = true;

                else
                    SpringsOn = false;
            };

            // Is there a swinging pendulum?
            ToggleButton PendulumsToggle = FindViewById<ToggleButton>(Resource.Id.PendulumToggle);
            PendulumsToggle.Click += (o, e) =>
            {
                if (PendulumsToggle.Checked)
                    PendulumsOn = true;

                else
                    PendulumsOn = false;
            };
            
            // Is the object moving in circular or non-circular motion?
            ToggleButton MotionType = FindViewById<ToggleButton>(Resource.Id.CircularMotionToggle);
            MotionType.Click += (o, e) =>
            {
                if (MotionType.Checked)
                {
                    CircularMotion = false;
                }

                else
                {
                    CircularMotion = true;
                }
            };
            
            // What is the mass of the main object?
            CheckBox mass = FindViewById<CheckBox>(Resource.Id.MassCheck);
            mass.Click += (o, e) => {
                if (mass.Checked)
                    HasMass = true;
                else
                    HasMass = false;
            };

            // Is there any linear acceleration?
            CheckBox LinearAcceleration = FindViewById<CheckBox>(Resource.Id.AccelerationCheck);
            LinearAcceleration.Click += (o, e) => {
                if (LinearAcceleration.Checked)
                {
                    LinAccel = true;
                    CircularMotion = false;
                }
                
                else
                {
                    LinAccel = false;
                    CircularMotion = true;
                }
            };
            
            // Is there a linear velocity?
            CheckBox LinearVelocity = FindViewById<CheckBox>(Resource.Id.VelocityCheck);
            LinearVelocity.Click += (o, e) => {
                if (LinearVelocity.Checked)
                    LinVel = true;
                else
                    LinVel = false;
            };

            // Is there an initial velocity?
            CheckBox InitialVelocityCheck = FindViewById<CheckBox>(Resource.Id.InitialVelocityCheck);
            checkbox.Click += (o, e) => {
                if (checkbox.Checked) ;
                // Stuff goes here!
                else;
                // Stuff goes here!
            };

            // Are you given anything about energy?
            CheckBox EnergyOn = FindViewById<CheckBox>(Resource.Id.TotEnergyCheck);
            EnergyOn.Click += (o, e) => {
                if (EnergyOn.Checked)
                    UsesEnergy == true;
                else
                    UsesEnergy == false;
            };
            
            CheckBox checkbox = FindViewById<CheckBox>(Resource.Id.checkbox);
            checkbox.Click += (o, e) => {
                if (checkbox.Checked) ;
                // Stuff goes here!
                else;
                // Stuff goes here!
            };

            CheckBox checkbox = FindViewById<CheckBox>(Resource.Id.checkbox);
            checkbox.Click += (o, e) => {
                if (checkbox.Checked) ;
                // Stuff goes here!
                else;
                // Stuff goes here!
            };
        }
    }
}
