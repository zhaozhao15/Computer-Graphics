#include"../inc/common.h"
#include "../inc/shader_m.h"

#define PPI 180
const float PI = 3.141593f;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);

const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 800;

// camera
GLdouble posx = 0;
GLdouble posy = 0;
GLdouble posz = 5;

GLdouble targetx = 0;
GLdouble targety = 0;
GLdouble targetz = 0;

GLdouble upx = 0;
GLdouble upy = 1;
GLdouble upz = 0;

GLfloat lastX;
GLfloat lastY;
GLfloat mousePitch = 0.0f;
GLfloat mouseYaw = 0.0f;
bool firstMouse = true;


void drawQuad()
{
	glBegin(GL_QUADS);

	glNormal3f(0.0f, 0.0f, 1.0f);
	glVertex3f(0.5f, 0.5f, 0.5f);
	glVertex3f(-0.5f, 0.5f, 0.5f);
	glVertex3f(-0.5f, -0.5f, 0.5f);
	glVertex3f(0.5f, -0.5f, 0.5f);

	glEnd();
}

void drawTri()
{
	glBegin(GL_TRIANGLES);

	glColor3f(0.6f, 0.2f, 0.3f);
	glVertex3f(0, 0, 1);
	glVertex3f(1, 0, 0);
	glVertex3f(0, 1, 0);

	glEnd();
}


void drawSphere(GLfloat xx, GLfloat yy, GLfloat zz, GLfloat radius, GLfloat M, GLfloat N)
{
	float step_z = PPI / M;           //z方向每次步进的角度  
	float step_xy = 2 * PPI / N;      //x,y平面每次步进的角度  
	float x[4], y[4], z[4];          //用来存坐标  

	float angle_z = 0.0;             //起始角度  
	float angle_xy = 0.0;
	int i = 0, j = 0;
	glBegin(GL_QUADS);
	for (i = 0; i<M; i++)
	{
		angle_z = i * step_z;        //每次步进step_z  

		for (j = 0; j<N; j++)
		{
			angle_xy = j * step_xy;  //每次步进step_xy  

			//一层一层的画出来  
			x[0] = radius * sin(angle_z) * cos(angle_xy);  //第一个小平面的第一个顶点坐标  
			y[0] = radius * sin(angle_z) * sin(angle_xy);
			z[0] = radius * cos(angle_z);

			x[1] = radius * sin(angle_z + step_z) * cos(angle_xy);  //第一个小平面的第二个顶点坐标，下面类似  
			y[1] = radius * sin(angle_z + step_z) * sin(angle_xy);
			z[1] = radius * cos(angle_z + step_z);

			x[2] = radius * sin(angle_z + step_z)*cos(angle_xy + step_xy);
			y[2] = radius * sin(angle_z + step_z)*sin(angle_xy + step_xy);
			z[2] = radius * cos(angle_z + step_z);

			x[3] = radius * sin(angle_z) * cos(angle_xy + step_xy);
			y[3] = radius * sin(angle_z) * sin(angle_xy + step_xy);
			z[3] = radius * cos(angle_z);
			//至此得到4个顶点  
			for (int k = 0; k<4; k++)
			{
				glNormal3f(xx + x[k], yy + y[k], zz + z[k]);
				glVertex3f(xx + x[k], yy + y[k], zz + z[k]);      //画出这个平面  
			}  
		}             //循环画出这一层的平面，组成一个环  
	}                             //z轴++，画出剩余层  
	glEnd(); 
}


void DrawObjectWithTransform()
{
	glPushMatrix();
	float rotate = glfwGetTime() * 30;
	glRotatef(rotate, 0.0, 1.0, 0.0);
	//draw triangle
	glBegin(GL_QUADS);
	glNormal3f(0.0f, 0.0f, 1.0f);
	glVertex3f(2.5f, 0.5f, 0.5f);
	glVertex3f(1.5f, 0.5f, 0.5f);
	glVertex3f(1.5f, -0.5f, 0.5f);
	glVertex3f(2.5f, -0.5f, 0.5f);

	glNormal3f(0.0f, 0.0f, -1.0f);
	glVertex3f(2.5f, 0.5f, -0.5f);
	glVertex3f(1.5f, 0.5f, -0.5f);
	glVertex3f(1.5f, -0.5f, -0.5f);
	glVertex3f(2.5f, -0.5f, -0.5f);
	
	glNormal3f(1.0f, 0.0f, 0.0f);
	glVertex3f(2.5f, 0.5f, -0.5f);
	glVertex3f(2.5f, 0.5f, 0.5f);
	glVertex3f(2.5f, -0.5f, 0.5f);
	glVertex3f(2.5f, -0.5f, -0.5f);

	glNormal3f(-1.0f, 0.0f, 0.0f);
	glVertex3f(1.5f, 0.5f, -0.5f);
	glVertex3f(1.5f, 0.5f, 0.5f);
	glVertex3f(1.5f, -0.5f, 0.5f);
	glVertex3f(1.5f, -0.5f, -0.5f);

	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(1.5f, 0.5f, -0.5f);
	glVertex3f(1.5f, 0.5f, 0.5f);
	glVertex3f(2.5f, 0.5f, 0.5f);
	glVertex3f(2.5f, 0.5f, -0.5f);

	glNormal3f(0.0f, -1.0f, 0.0f);
	glVertex3f(1.5f, -0.5f, -0.5f);
	glVertex3f(1.5f, -0.5f, 0.5f);
	glVertex3f(2.5f, -0.5f, 0.5f);
	glVertex3f(2.5f, -0.5f, -0.5f);
	
	
	glEnd();


	glPopMatrix();
}


void initPMV()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(60, SCR_WIDTH / SCR_HEIGHT, 0.1, 100);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt
	(
		posx, posy, posz,
		targetx, targety, targetz,
		upx, upy, upz
	);

}




void ChangePmv(float deltaX, float deltaY)
{
	mouseYaw += deltaX;
	mousePitch += deltaY;
	if (mousePitch > 89.0f)
		mousePitch = 89.0f;
	if (mousePitch < -89.0f)
		mousePitch = -89.0f;
	targetx = cos(mousePitch / 180.0f*PI) * cos(mouseYaw / 180.0f*PI);
	targety = sin(mousePitch / 180.0f*PI);
	targetz = cos(mousePitch / 180.0f*PI) * sin(mouseYaw / 180.0f*PI);
	
	initPMV();
}


int main(int argc, char* argv[])
{
	glfwInit();

	glEnable(GL_MULTISAMPLE);

	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}


	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	
	glewExperimental = GL_TRUE;
	// Initialize GLEW to setup the OpenGL Function pointers
	glewInit();
	
	initPMV();

	
	//开灯
	GLfloat sun_light_position[] = { 5.0f, 5.0f, 5.0f, 1.0f };
	GLfloat sun_light_ambient[] = { 0.5f, 0.5f, 0.5f, 1.0f };
	GLfloat sun_light_diffuse[] = { 0.5f, 1.0f, 0.5f, 1.0f };
	GLfloat sun_light_specular[] = { 1.0f, 0.0f, 1.0f, 1.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, sun_light_position);
	glLightfv(GL_LIGHT0, GL_AMBIENT, sun_light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, sun_light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, sun_light_specular);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
	//材质
	GLfloat earth_mat_ambient[] = { 0.0f, 0.0f, 1.0f, 1.0f };
	GLfloat earth_mat_diffuse[] = { 0.0f, 1.0f, 0.5f, 1.0f };
	GLfloat earth_mat_specular[] = { 1.0f, 0.0f, 1.0f, 1.0f };
	GLfloat earth_mat_emission[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	GLfloat earth_mat_shininess = 30.0f;
	glMaterialfv(GL_FRONT, GL_AMBIENT, earth_mat_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, earth_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, earth_mat_specular);
	glMaterialfv(GL_FRONT, GL_EMISSION, earth_mat_emission);
	glMaterialf(GL_FRONT, GL_SHININESS, earth_mat_shininess);

	glEnable(GL_NORMALIZE);


	while (!glfwWindowShouldClose(window))
	{
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

		processInput(window);
		glClearColor(0.3,0.7,0.5,0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		
		
		drawSphere(0,  0,  0,  0.6,  300,  300);
		DrawObjectWithTransform();

		glfwSetCursorPosCallback(window, mouse_callback);


		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	// glfw: terminate, clearing all previously allocated GLFW resources.
	// ------------------------------------------------------------------
	glfwTerminate();
	return 0;
	

}

void processInput(GLFWwindow *window)
{
	GLfloat unitchange = 0.01f;
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
	{
		glfwSetWindowShouldClose(window, true);
	}
	else {
		if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS)
		{
			posy += unitchange;
			targety += unitchange;
		}
		if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS)
		{
			posy -= unitchange;
			targety -= unitchange;
		}
		if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		{
			posx -= unitchange;
			targetx -= unitchange;
		}
		if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		{
			posx += unitchange;
			targetx += unitchange;
		}
		if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		{
			posz -= unitchange;
			targetz -= unitchange;
		}
		if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		{
			posz += unitchange;
			targetz += unitchange;
		}
		initPMV();
	}



	
	
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}



void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (firstMouse)
	{
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
	}

	float xoffset = xpos - lastX;
	float yoffset = lastY - ypos;

	float sensitivity = 0.3f;
	xoffset *= sensitivity;
	yoffset *= sensitivity;

	lastX = xpos;
	lastY = ypos;

	ChangePmv(xoffset, yoffset);
}

