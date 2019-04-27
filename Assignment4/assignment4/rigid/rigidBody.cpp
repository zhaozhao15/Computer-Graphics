#include "rigidBody.h"

RigidBody::RigidBody()
{
	Vector3d center(0, 20, 0);
	vertexPos[0] = Vector3d(-halfSize, -halfSize, halfSize) + center;
	vertexPos[1] = Vector3d(halfSize, -halfSize, halfSize) + center;
	vertexPos[2] = Vector3d(halfSize, -halfSize, -halfSize) + center;
	vertexPos[3] = Vector3d(-halfSize, -halfSize, -halfSize) + center;
	vertexPos[4] = Vector3d(-halfSize, halfSize, halfSize) + center;
	vertexPos[5] = Vector3d(halfSize, halfSize, halfSize) + center;
	vertexPos[6] = Vector3d(halfSize, halfSize, -halfSize) + center;
	vertexPos[7] = Vector3d(-halfSize, halfSize, -halfSize) + center;

	for (int i = 0; i < number; i++) {
		vertexPosNew[i] = vertexPos[i];
	}

	Matrix3x3 m(1, 0, 0, 0, 1, 0, 0, 0, 1);
	rigidState.xposition = center;
	rigidState.quater = Quaternion(m);
	rigidState.pfmom = Vector3d(0, 0, 0);
	rigidState.lamom = Vector3d(0, 0, 0);

	Vector3d w(0, 0, 0);
	rigidStateDot.velocity = Vector3d(0, 0, 0);
	rigidStateDot.quaterA = 0.5*w*(rigidState.quater);
	rigidStateDot.force = Vector3d(0, 0, 0);
	rigidStateDot.torque = Vector3d(0, 0, 0);

}

RigidBody::~RigidBody()
{
}

void RigidBody::startFall()
{
	resetSign = false;
	bodyForce = Vector3d(0, -100, 0);
}

Matrix3x3 RigidBody::getStateRotation()
{
	Matrix3x3 m = rigidState.quater.rotation();
	return m;
}
 
Vector3d RigidBody::getStatePosition()
{
	return rigidState.xposition;
}

void RigidBody::updateFall()
{
	unsigned int counter = 0;
	for (int i = 0; i<number; i++) {
		if (vertexPosNew[i].y < 0) {
			counter++;
			return;
		}
	}
	
	double hStep = 0.01;

	statesNumInt(rigidState, rigidStateDot,	rigidStateNew, hStep);
	collisionDetect(rigidState, rigidStateDot,	rigidStateNew, hStep);

	for (int i = 0; i<number; i++) {
		vertexForce[i] = Vector3d(0, 0, 0);
	}

	rigidState = rigidStateNew;
	center = rigidState.xposition;
	for (int i = 0; i<number; i++) {
		vertexPos[i] = vertexPosNew[i];
	}
}

StateDot RigidBody::F(Rigidstate& rigidState)// compute force
{
	StateDot rigidStateDot;

	rigidStateDot.velocity = (1.0 / mass)*rigidState.pfmom;
	Matrix3x3 R = rigidState.quater.rotation();
	Matrix3x3 Iinverse = R * (Io.inv()) * (R.transpose());
	Vector3d w = Iinverse * rigidState.lamom;
	rigidStateDot.quaterA = 0.5 * w * rigidState.quater;

	Vector3d totalforce(0, 0, 0);
	for (int i = 0; i<number; i++) {
		totalforce = totalforce + vertexForce[i];
	}
	totalforce = totalforce + bodyForce;
	rigidStateDot.force = totalforce;

	Vector3d totalTorque(0, 0, 0);
	Vector3d torque(0, 0, 0);
	for (int i = 0; i<number; i++) {
		torque = (vertexPos[i] - center) % vertexForce[i];
		totalTorque = totalTorque + torque;
	}
	rigidStateDot.torque = totalTorque;

	return rigidStateDot;
}

void RigidBody::statesNumInt(Rigidstate& rigidState, StateDot& rigidStateDot, Rigidstate& rigidStateNew, double h)
{
	StateDot K1 = F(rigidState);
	rigidStateDot = K1;
	rigidStateNew = rigidState + rigidStateDot * (h);

	Vector3d centerNew = rigidStateNew.xposition;
	Matrix3x3 R = rigidStateNew.quater.rotation();

	vertexPosNew[0] = R * (Vector3d(-halfSize, -halfSize, halfSize)) + centerNew;
	vertexPosNew[1] = R * (Vector3d(halfSize, -halfSize, halfSize)) + centerNew;
	vertexPosNew[2] = R * (Vector3d(halfSize, -halfSize, -halfSize)) + centerNew;
	vertexPosNew[3] = R * (Vector3d(-halfSize, -halfSize, -halfSize)) + centerNew;
	vertexPosNew[4] = R * (Vector3d(-halfSize, halfSize, halfSize)) + centerNew;
	vertexPosNew[5] = R * (Vector3d(halfSize, halfSize, halfSize)) + centerNew;
	vertexPosNew[6] = R * (Vector3d(halfSize, halfSize, -halfSize)) + centerNew;
	vertexPosNew[7] = R * (Vector3d(-halfSize, halfSize, -halfSize)) + centerNew;
}

void RigidBody::collisionDetect(Rigidstate& rigidState, StateDot& rigidStateDot, Rigidstate& rigidStateNew, double h)
{
	for (int i = 0; i<number; i++) {
		if ((vertexPosNew[i].y - wallY)*(vertexPos[i].y - wallY)<0) { //下一时刻即将穿透
			if ((vertexPosNew[i] - vertexPos[i]).norm() > 0.1) {
				while (1)
				{
					statesNumInt(rigidState, rigidStateDot, rigidStateNew, h*0.5);
					if (vertexPosNew[i].y <wallY) {
						break;
					}
					else
					{
						rigidState = rigidStateNew;
						center = rigidState.xposition;
						for (int i = 0; i<number; i++) {
							vertexPos[i] = vertexPosNew[i];
						}
					}
				}
				collisionDetect(rigidState, rigidStateDot, rigidStateNew, h*0.5);
			}
			else
			{
				//resting
				if (rigidStateDot.velocity.norm()< thr) {
					unsigned int counter = 0;
					for (int i = 0; i<number; i++) {
						if (vertexPosNew[i].y < thr) {
							counter++;
							break;
						}
					}
				}

				Matrix3x3 R = rigidState.quater.rotation();
				Matrix3x3 Iinverse = R * (Io.inv()) * (R.transpose());
				Vector3d w = Iinverse * rigidState.lamom;
				Vector3d r = vertexPos[i] - center;

				double Vn = Vpn * (rigidStateDot.velocity + w % r);
				double j = -1 * (1 + Cr)*Vn / (1.0 / mass + Vpn * (Iinverse*(r%Vpn) % r));
				Vector3d J = j * Vpn;

				rigidState.pfmom = rigidState.pfmom + J;
				rigidState.lamom = rigidState.lamom + r % J;

				statesNumInt(rigidState, rigidStateDot, rigidStateNew, h);
			}
		}
	}
}

void RigidBody::ResolveCollisions(int cornerIndex)
{
	
}
