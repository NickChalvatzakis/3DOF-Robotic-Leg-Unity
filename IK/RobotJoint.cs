using UnityEngine;

namespace RobotJoints {
    public class RobotJoint: MonoBehaviour {
        public Vector3 Axis;
        public Vector3 StartOffset;

        public float MinAngle;
        public float MaxAngle;

        // The initial one
        public Vector3 ZeroEuler;

        void Awake() {
            ZeroEuler = transform.localEulerAngles;
            StartOffset = transform.localPosition;
        }

        // Try to move the angle by delta.
        // Returns the new angle.
        public float ClampAngle(float angle, float delta = 0) {
            return Mathf.Clamp(angle + delta, MinAngle, MaxAngle);
        }

        // Get the current angle
        public float GetAngle() {
            float angle = 0;
            if (Axis.x == 1) angle = transform.localEulerAngles.x;
            else
            if (Axis.y == 1) angle = transform.localEulerAngles.y;
            else
            if (Axis.z == 1) angle = transform.localEulerAngles.z;

            return ClampAngle(angle);
        }
        public float SetAngle(float angle) {
            angle = ClampAngle(angle);
            if (Axis.x == 1) transform.localEulerAngles = new Vector3(angle, 0, 0);
            else
            if (Axis.y == 1) transform.localEulerAngles = new Vector3(0, angle, 0);
            else
            if (Axis.z == 1) transform.localEulerAngles = new Vector3(0, 0, angle);

            return angle;
        }



        // Moves the angle to reach 
        public float MoveArm(float angle) {
            return SetAngle(angle);
        }

        private void OnDrawGizmos() {
            Debug.DrawLine(transform.position, transform.parent.position, Color.red);
        }
    }


}