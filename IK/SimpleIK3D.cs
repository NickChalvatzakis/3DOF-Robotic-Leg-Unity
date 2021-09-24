using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Alan Zucconi
// Inverse Kinematics 3D
// https://www.alanzucconi.com/?p=12166
namespace AlanZucconi.IK
{
    public class SimpleIK3D : MonoBehaviour
    {
        struct IKResult
        {
            // theta
            public float AngleT;

            public float Angle0;
            public float Angle1;   
        }

        [Header("Joints")]
        public Transform Joint0;
        public Transform Joint1;
        public Transform Hand;

        [Header("Target")]
        public Transform Target;

        // Update is called once per frame
        void Update()
        {
            IKResult result;
            IK(out result);
            {
                Vector3 Euler0 = Joint0.transform.localEulerAngles;
                Euler0.z = result.Angle0;
                Euler0.y = result.AngleT;
                Joint0.transform.localEulerAngles = Euler0;

                Vector3 Euler1 = Joint1.transform.localEulerAngles;
                Euler1.z = result.Angle1;
                Joint1.transform.localEulerAngles = Euler1;
            }
        }
        
        private bool IK (out IKResult result)
        {
            Vector3 A = Joint0.position;
            Vector3 B = Joint1.position;
            Vector3 C = Target.position;

            Vector3 H = Hand.position;


            float length_AB = Vector3.Distance(A, B); // 0
            float length_BC = Vector3.Distance(B, H); // 1
            float length_AC = Vector3.Distance(A, C); // 2

            // The target point, but projected on the XZ plane
            Vector3 C0 = C;
            C0.y = A.y;
            //C0.y = 0;

            float length_CC0 = Vector3.Distance(C, C0);

            float dx = Vector3.Distance(C0, A);
            float dy = C.y - C0.y;
            float Aprime = Mathf.Atan2(dy, dx); // rad
            
            Vector3 diffXZ = C0 - A;
            float theta = -Mathf.Atan2(diffXZ.z, diffXZ.x); // rad

            result = new IKResult();
            
            // Is the target reachable?
            // If not, we stretch as far as possible
            if (length_AB + length_BC < length_AC)
            {
                result.AngleT = theta * Mathf.Rad2Deg; // deg
                result.Angle0 = Aprime * Mathf.Rad2Deg; // deg
                result.Angle1 = 0f; // deg
                return false;
            }
            
            

            float cosAngle0 = ((length_AC * length_AC) + (length_AB * length_AB) - (length_BC * length_BC)) / (2 * length_AC * length_AB); // rad
            float angle0 = Mathf.Acos(cosAngle0); // rad

            float cosAngle1 = ((length_BC * length_BC) + (length_AB * length_AB) - (length_AC * length_AC)) / (2 * length_BC * length_AB); // rad
            float angle1 = Mathf.Acos(cosAngle1); // rad

            // So they work in Unity reference frame
            result.AngleT = theta * Mathf.Rad2Deg; // deg
            result.Angle0 = (Aprime + angle0) * Mathf.Rad2Deg; // deg
            result.Angle1 = (Mathf.PI + angle1) * Mathf.Rad2Deg; // deg

            return true;
        }
    }
}
