//============================================================================
// Name        : RungeKutta4.cpp
// Author      : werasaimon
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <vector>
#include <math.h>
#include "freeglut/glut.h"


using namespace std;



#define GLUT_WHEEL_UP   3
#define GLUT_WHEEL_DOWN 4

#define EPSILON 0.0001

class Vector3
{

public:

	//--------------------[ Attributes ]--------------------//

	float x;
	float y;
	float z;


public:

        //----------------[ constructors ]--------------------------

    Vector3()
    : x(0), y(0), z(0)
    {
    }

    Vector3(float nx, float ny, float nz)
     : x(nx), y(ny), z(nz)
    {
    }



    //---------------[ vector arithmetic operator ]--------------

    Vector3 operator+(const Vector3& rhs) const
    {
        return Vector3(x + rhs.x, y + rhs.y, z + rhs.z);
    }


    Vector3 operator-(const Vector3& rhs) const
    {
        return Vector3(x - rhs.x, y - rhs.y, z - rhs.z);
    }

    Vector3 operator*(const Vector3& rhs) const
    {
        return Vector3(x * rhs.x, y * rhs.y, z * rhs.z);
    }

    Vector3 operator/(const Vector3& rhs) const
    {
        return Vector3(x / rhs.x, y / rhs.y, z / rhs.z);
    }

    //--------------[ scalar vector operator ]--------------------

    Vector3 operator*(float rhs) const
    {
        return Vector3(x * rhs, y * rhs, z * rhs);
    }

    Vector3 operator/(float rhs) const
    {
        return Vector3(x / rhs, y / rhs, z / rhs);
    }


    //-------------[ unary operations ]--------------------------

    Vector3 operator-() const
    {
        return Vector3(-x, -y, -z);
    }



    //-------------[ function operations ]---------------------------

    float dot(const Vector3& rhs) const
    {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    Vector3 cross(const Vector3& rhs) const
    {
        return Vector3(y * rhs.z - rhs.y * z,
                       z * rhs.x - rhs.z * x,
                       x * rhs.y - rhs.x * y);
    }

    float length() const
    {
        return sqrt(x * x + y * y + z * z);
    }

    float lengthSquare() const
    {
        return x * x + y * y + z * z;
    }

    void normalize()
    {
        float s = length();
        x /= s;
        y /= s;
        z /= s;
    }

    Vector3 getUnit() const
    {
        float lengthVector = length();
        if (lengthVector < EPSILON)
        {
            return *this;
        }
        // Compute and return the unit vector
        float lengthInv = 1.0 / lengthVector;
        return Vector3( x * lengthInv,
                        y * lengthInv,
                        z * lengthInv);
    }


    void rotateAroundAxis(const Vector3 &axis, const float angle)
    {
    	Vector3 self = *this;
    	Vector3 x = self - axis * (axis * self);
    	Vector3 y = x.cross(axis);
    	Vector3 delta = x * (cos(angle)) + y * (sin(angle)) - x;
    	self = self + delta;
    	*this = self;
    }
};



enum BodyType {STATIC , DYNAMIC};

struct TParticle
{

	//--------------------[ Attributes ]--------------------//

	BodyType mType;

	Vector3  mPosition;
	Vector3  mVelocity;

	Vector3  mSimPosition;
	Vector3  mSimVelocity;

	Vector3  mVels[4];
	Vector3  mForces[4];

	float    mMassInverse;


	//-------------------[ constructors ]--------------------//

	TParticle( const Vector3& _pos , float _massa , BodyType _type )
	: mPosition(_pos) , mMassInverse(1.0 / _massa) , mType(_type)
	{

	}
};



struct TJoint
{
	//--------------------[ Attributes ]--------------------//

	TParticle *Particle1;
	TParticle *Particle2;

	float mIdealDist;
	float mRelaxation;


	//-------------------[ constructors ]--------------------//


	TJoint( TParticle *_particle1  , TParticle *_particle2 , const float &Dist)
	{
		Particle1   = _particle1;
		Particle2   = _particle2;
		mIdealDist  =  Dist;
		mRelaxation = 0.01;
	}

	TJoint( TParticle *_particle1  , TParticle *_particle2 )
	{
		Particle1   = _particle1;
		Particle2   = _particle2;
		mIdealDist  = (_particle1->mPosition - _particle2->mPosition ).length();
		mRelaxation = 0.01;
	}



};

struct TPhysicsGridSimulate
{

	//--------------------[ Attributes ]--------------------//

	Vector3 mGravity;

	std::vector<TParticle*> mParticles;
	std::vector<TJoint*>    mJoints;


	void AddParticle( TParticle *part )
	{
		mParticles.push_back(part);
	}

	void AddJoint( TJoint *joint )
	{
		mJoints.push_back(joint);
	}

	//--------------------[ constructors ]--------------------//


	TPhysicsGridSimulate( const Vector3& _gravity )
	: mGravity(_gravity)
	{

	}


	void Solver( int const& Step )
	{

		const float TensionConstant = 600.f;
		const float TensionDampingConstant = 2.0f;


		// Add gravity, store velocity
		for( unsigned int i = 0; i < mParticles.size(); ++i )
		{
			if( mParticles[i]->mType == DYNAMIC ) mParticles[i]->mForces[Step] = mGravity;
			if( mParticles[i]->mType == DYNAMIC ) mParticles[i]->mVels[Step]   = mParticles[i]->mSimVelocity;
		}


		/**/
		// Then further in Forces [Step] it is necessary to throw all the forces acting on particles.
		// Below, for example, the forces of springs are added.

		// Add spring forces
		for( unsigned int i = 0; i < mJoints.size(); ++i )
		{
			TParticle *PParticle1 = mJoints[i]->Particle1;
			TParticle *PParticle2 = mJoints[i]->Particle2;

			Vector3 PosDelta = PParticle1->mSimPosition - PParticle2->mSimPosition;
			Vector3 VelDelta = PParticle1->mSimVelocity - PParticle2->mSimVelocity;

			float  Dist = PosDelta.length();

			if( fabs(Dist) > EPSILON )
			{
				float F = -(TensionConstant * (Dist - mJoints[i]->mIdealDist) + TensionDampingConstant * ((PosDelta.dot(VelDelta)) / Dist));
				Vector3 Force = PosDelta / Dist * F;

				//mJoints[i]->mAccumulated[Step] += Force;

				if( PParticle1->mType == DYNAMIC ) PParticle1->mForces[Step] = PParticle1->mForces[Step] + Force * PParticle1->mMassInverse;
				if( PParticle2->mType == DYNAMIC ) PParticle2->mForces[Step] = PParticle2->mForces[Step] - Force * PParticle2->mMassInverse;

				float relax = 1.0 - mJoints[i]->mRelaxation;

				PParticle1->mVels[Step] =  PParticle1->mVels[Step] * relax;
				PParticle2->mVels[Step] =  PParticle2->mVels[Step] * relax;

			}
		}
		/**/


	}


	void UpdateRungeKutta4(float TimeStep)
	{
		int iter_rk4 = 0;
		for( unsigned int i = 0; i < mParticles.size(); ++i )
		{
			TParticle *Part = mParticles[i];
			Part->mSimPosition = Part->mPosition;
			Part->mSimVelocity = Part->mVelocity;
		}

		Solver(iter_rk4);
		iter_rk4++;

		for( unsigned int i = 0; i < mParticles.size(); ++i )
		{
			TParticle *Part = mParticles[i];
			Part->mSimPosition = Part->mPosition + (Part->mVels[0]   * TimeStep/2);
			Part->mSimVelocity = Part->mVelocity + (Part->mForces[0] * TimeStep/2);
		}

		Solver(iter_rk4);
		iter_rk4++;

		for( unsigned int i = 0; i < mParticles.size(); ++i )
		{
			TParticle *Part = mParticles[i];
			Part->mSimPosition = Part->mPosition + (Part->mVels[1] * TimeStep/2);
			Part->mSimVelocity = Part->mVelocity + (Part->mForces[1] * TimeStep/2);
		}

		Solver(iter_rk4);
		iter_rk4++;

		for( unsigned int i = 0; i < mParticles.size(); ++i )
		{
			TParticle *Part = mParticles[i];
			Part->mSimPosition = Part->mPosition + (Part->mVels[2] * TimeStep);
			Part->mSimVelocity = Part->mVelocity + (Part->mForces[2] * TimeStep);
		}

		Solver(iter_rk4);


		for( unsigned int i = 0; i < mParticles.size(); ++i )
		{
			TParticle *Part = mParticles[i];
			if(Part->mType == DYNAMIC)
			{
			    Part->mPosition = RungeKuttaSum(Part->mPosition, Part->mVels[0], Part->mVels[1], Part->mVels[2], Part->mVels[3], TimeStep);
		    	Part->mVelocity = RungeKuttaSum(Part->mVelocity, Part->mForces[0], Part->mForces[1], Part->mForces[2], Part->mForces[3], TimeStep);
			}

		}

	}

	static Vector3 RungeKuttaSum(const Vector3& Pos, const Vector3& K1,const Vector3&  K2, const Vector3&  K3, const Vector3&  K4 , float const& TimeStep)
	{
		return  Pos + (K1 + K2 * 2.0 + K3 * 2.0 + K4) * TimeStep / 6.f;
	}


    /***************************  Display Render *************************/
	void Render()
	{
		for( unsigned int i = 0; i < mParticles.size(); ++i )
		{
			TParticle part = *mParticles[i];
			glPushMatrix();
			glColor3f(0,1,0);
			glPointSize(2.0);
			glBegin(GL_POINTS);
			glVertex3f(part.mPosition.x, part.mPosition.y, part.mPosition.z);
			glEnd();
			glPopMatrix();

		}

		for ( unsigned int i = 0; i < mJoints.size(); ++i)
		{
			Vector3 pos1 = mJoints[i]->Particle1->mPosition;
			Vector3 pos2 = mJoints[i]->Particle2->mPosition;
			glPushMatrix();
			glColor3f(0,1,0);
			glBegin(GL_LINES);
			glVertex3f( pos1.x , pos1.y , pos1.z );
			glVertex3f( pos2.x , pos2.y , pos2.z );
			glEnd();
			glPopMatrix();
		}

	}

};



//=================================  Unit Test ==================================//


/// Camera value
Vector3 mEye;
Vector3 mCenter;
Vector3 mUp;

float Width = 600;
float Height = 400;

/// Physics value
const Vector3 gravity(0,-20,0);
TPhysicsGridSimulate DynamicGrid(gravity);

float TimeStep = 1.0 / 60.0;

// Initialization
void initCamera()
{

	float aspect = Width / Height;
	float zNear  = 1.0;
	float zFar   = 1024;
	float fov    = 45.0;

	mEye    =  Vector3(0,0,0);
	mCenter =  Vector3(0,0,0);
	mUp     =  Vector3(0,1,0);


	glLoadIdentity();

	glMatrixMode(GL_PROJECTION);
	gluPerspective( fov , aspect , zNear , zFar );

	glMatrixMode(GL_MODELVIEW);
	//	  gluLookAt(0.0, 0.0, 5.0,  /* eye is at (0,0,5) */
	//	    0.0, 0.0, 0.0,      /* center is at (0,0,0) */
	//	    0.0, 1.0, 0.);      /* up is in positive Y direction */

}

void InitPhysicsParticles()
{

	const unsigned int _coutNb = 50;

	TParticle *prevPart;
	TParticle *prevParts[_coutNb];

	float step_grid = 1.0 / 2.0;

	for (unsigned int j = 0; j < _coutNb; ++j)
	{
		for (unsigned int i = 0; i < _coutNb; ++i)
		{
		   float massa = 1.0;

		   Vector3 Position = Vector3(_coutNb * step_grid * 0.25 - i * step_grid * 0.5 , -2 ,
				                      _coutNb * step_grid * 0.25 - j * step_grid * 0.5);
		   BodyType Type;

		   if( i == 0 || i == _coutNb - 1 || j == 0 || j == _coutNb - 1 ) Type = STATIC; else Type = DYNAMIC;

		   TParticle *part = new TParticle(Position , massa , Type);

		   if( i > 0 )
		   {
			   DynamicGrid.AddJoint( new TJoint(part , prevPart));
		   }

		   if( j > 0 )
		   {
			   DynamicGrid.AddJoint( new TJoint(part , prevParts[i]));
		   }

		   DynamicGrid.AddParticle(part);
		   prevPart = part;

		   prevParts[i] = part;
		}

	}

}



// Display the scene
void display()
{

	glViewport(0, 0, Width , Height );
	glLoadIdentity();

	//	float aspect = 1.0;//Width / Height;
	//	float zNear  = 1.0;
	//	float zFar   = 1024;
	//	float fov    = 45.0;
	//
	//	glMatrixMode(GL_PROJECTION);
	//	gluPerspective( fov , aspect , zNear , zFar );


	glMatrixMode(GL_MODELVIEW);
	gluLookAt( -mEye.x , -mEye.y , -mEye.z ,  mCenter.x , mCenter.y , mCenter.z , mUp.x , mUp.y , mUp.z );



	glEnable(GL_DEPTH_TEST);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	/// Render Physics Particles
	DynamicGrid.Render();


	glutSwapBuffers();


}






float zoom_distance = 25;

float oldX = 0;
float oldY = 0;

float angle_X = 0;
float angle_Y = 0;

int   mouse_button = -1;



void UpdateTime(void)
{

	//**********  Camera Position *******//
	Vector3 DircetionLeft(1,0,0);
	Vector3 DircetionLook(0,0,zoom_distance);
	DircetionLook.rotateAroundAxis(Vector3(0,1,0) , angle_X);
	DircetionLeft.rotateAroundAxis(Vector3(0,1,0) , angle_X);
	DircetionLook.rotateAroundAxis(DircetionLeft  , angle_Y);
	mEye = DircetionLook;//+ Vector3(-10,-15,0);


	//************ Render Scene *********//
	display();


	//************ Update Physics *******//
    DynamicGrid.UpdateRungeKutta4(TimeStep);

};


// Reshape function
void reshape(int width, int height)
{
	Width  = width;
	Height = height;
}


// Called when a mouse button event occurs
void mouseButton(int button, int state, int x, int y)
{
	mouse_button = button;

    float aspect = Width / Height;
    float m_x = ((x / Width ) - 0.5f) * aspect * 0.834;
    float m_y = ((y / Height) - 0.5f) * 0.834;

    oldX = m_x;
    oldY = m_y;

    if (state == GLUT_UP )
    {
    	if ( button == GLUT_WHEEL_UP )
    	{
    		zoom_distance -= 0.5;
    	}
    	else if( button == GLUT_WHEEL_DOWN )
    	{
    		zoom_distance += 0.5;
    	}
    }


    if (button == GLUT_RIGHT_BUTTON )
	{
        for (unsigned int i = 0; i < DynamicGrid.mParticles.size(); ++i)
        {
        	float rand_x = (100 - rand()%200);
        	float rand_y = (100 - rand()%200);
        	float rand_z = (100 - rand()%200);
        	cout<< i << " ) suka-random: " << rand_x << "  " << rand_y << "  " << rand_z <<endl;
        	DynamicGrid.mParticles[i]->mVelocity = DynamicGrid.mParticles[i]->mVelocity +  Vector3( rand_x , rand_y , rand_z ) * 0.1;
		}
	}
}

// Called when a mouse motion event occurs
void mouseMotion(int x, int y)
{
   float aspect = Width / Height;
   float m_x = ((x / Width ) - 0.5f) * aspect * 0.834;
   float m_y = ((y / Height) - 0.5f) * 0.834;

   float speedX = (m_x - oldX);
   float speedY = (m_y - oldY);

   if( mouse_button == GLUT_LEFT_BUTTON )
   {
      float coff = 0.5f;
      angle_X += speedX * coff;
      angle_Y += speedY * coff;
   }

   oldX = m_x;
   oldY = m_y;

}






// Main function
int main(int argc, char** argv)
{
	// Initialize GLUT
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    // Initialize the size of the GLUT windows
	glutInitWindowSize( Width ,  Height );
	// Initialize the position of the GLUT windows
	glutInitWindowPosition( 0 , 0 );
	// Initialize the create window
	glutCreateWindow("Demo: Runge-Kutta-4");

	initCamera();
	InitPhysicsParticles();

	glutDisplayFunc(display);

	// Glut Idle function that is continuously called
	glutReshapeFunc(reshape);
	glutMouseFunc(mouseButton);
	glutMotionFunc(mouseMotion);


	glutIdleFunc(UpdateTime);

	// Glut main looop
	glutMainLoop();


	/**/
	return 0;
}













