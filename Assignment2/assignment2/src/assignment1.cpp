#include "../bezier/mlbezier.h"
#include "../inc/shader_m.h"
#include "../3rdparty/stb_image.h"

const float PI = 3.141593f;

void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void processInput(GLFWwindow *window);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);

bool dragFlag = false;
float deltaTime = 0.0f; // 当前帧与上一帧的时间差
float lastFrame = 0.0f; // 上一帧的时间
mlBezier mlbezier;

const unsigned int SCR_WINDTH = 800;
const unsigned int SCR_HEIGHT = 600;


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


GLdouble *normalize(GLdouble *p1, GLdouble *p2, GLdouble *p3) {
	GLdouble *normal = new GLdouble[3];
	GLdouble x = (p2[1] - p1[1])*(p3[2] - p1[2]) - (p2[2] - p1[2])*(p3[1] - p1[1]);
	GLdouble y = (p2[2] - p1[2])*(p3[0] - p1[0]) - (p2[0] - p1[0])*(p3[2] - p1[2]);
	GLdouble z = (p2[0] - p1[0])*(p3[2] - p1[2]) - (p2[2] - p1[2])*(p3[0] - p1[0]);
	normal[0] = x;
	normal[1] = y;
	normal[2] = z;
	return normal;
}
 
void drawControlPoint(mlBezier &mlbezier)
{
	glBegin(GL_QUADS);
	for (int i = 0; i < (int)mlbezier.indicesofControlpoints.size() / 4; i++)
	{
		//glColor3f(0, 1, 0);
		glVertex3f
		(
			mlbezier.controlPoints[mlbezier.indicesofControlpoints[4 * i]].x,
			mlbezier.controlPoints[mlbezier.indicesofControlpoints[4 * i]].y,
			mlbezier.controlPoints[mlbezier.indicesofControlpoints[4 * i]].z
		);
		glVertex3f
		(
			mlbezier.controlPoints[mlbezier.indicesofControlpoints[4 * i + 1]].x,
			mlbezier.controlPoints[mlbezier.indicesofControlpoints[4 * i + 1]].y,
			mlbezier.controlPoints[mlbezier.indicesofControlpoints[4 * i + 1]].z
		);
		glVertex3f
		(
			mlbezier.controlPoints[mlbezier.indicesofControlpoints[4 * i + 2]].x,
			mlbezier.controlPoints[mlbezier.indicesofControlpoints[4 * i + 2]].y,
			mlbezier.controlPoints[mlbezier.indicesofControlpoints[4 * i + 2]].z
		);
		glVertex3f
		(
			mlbezier.controlPoints[mlbezier.indicesofControlpoints[4 * i + 3]].x,
			mlbezier.controlPoints[mlbezier.indicesofControlpoints[4 * i + 3]].y,
			mlbezier.controlPoints[mlbezier.indicesofControlpoints[4 * i + 3]].z
		);
	}
	glEnd();
}
void drawBezierSurface(mlBezier &mlbezier)
{
	glm::vec3 Points[100 * 100];
	int i = 0;
	for (int u = 0; u < 100; u++) {
		for (int v = 0; v < 100; v++) {
			Points[i] = mlbezier.mlEvalBezierPatch(mlbezier.controlPoints, (float)v / 100, (float)u / 100);
			i++;
		}
	}

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glBegin(GL_QUADS);
	
	for (int i = 0; i < (int)mlbezier.indicesofP.size() / 4; i++) {
		GLdouble PointA[] = { Points[mlbezier.indicesofP[4 * i]].x, Points[mlbezier.indicesofP[4 * i]].y, Points[mlbezier.indicesofP[4 * i]].z };
		GLdouble PointB[] = { Points[mlbezier.indicesofP[4 * i + 1]].x, Points[mlbezier.indicesofP[4 * i + 1]].y, Points[mlbezier.indicesofP[4 * i + 1]].z };
		GLdouble PointC[] = { Points[mlbezier.indicesofP[4 * i + 2]].x, Points[mlbezier.indicesofP[4 * i + 2]].y, Points[mlbezier.indicesofP[4 * i + 2]].z };

		glNormal3f(normalize(PointA, PointB, PointC)[0], normalize(PointA, PointB, PointC)[1], normalize(PointA, PointB, PointC)[2]);

		glTexCoord2f((float)(i%100)/100 + (float)(i / 100) / 100, (float)(i/100)/100);

		glVertex3f
		(
			Points[mlbezier.indicesofP[4 * i]].x,
			Points[mlbezier.indicesofP[4 * i]].y,
			Points[mlbezier.indicesofP[4 * i]].z
		);
		glVertex3f
		(
			Points[mlbezier.indicesofP[4 * i + 1]].x,
			Points[mlbezier.indicesofP[4 * i + 1]].y,
			Points[mlbezier.indicesofP[4 * i + 1]].z
		);
		glVertex3f
		(
			Points[mlbezier.indicesofP[4 * i + 2]].x,
			Points[mlbezier.indicesofP[4 * i + 2]].y,
			Points[mlbezier.indicesofP[4 * i + 2]].z
		);
		glVertex3f
		(
			Points[mlbezier.indicesofP[4 * i + 3]].x,
			Points[mlbezier.indicesofP[4 * i + 3]].y,
			Points[mlbezier.indicesofP[4 * i + 3]].z
		);
	}

	glEnd();
}

void initPMV()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(60, SCR_WINDTH / SCR_HEIGHT, 0.1, 100);

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


void initTexture(unsigned int &texture)
{
	int width, height, nrChannels;
	
	unsigned char *data = stbi_load("C:\\Users\\赵冬昊\\Desktop\\Computer Graphics\\Assignment2\\assignment2\\resource\\textures\\img.jpg", &width, &height, &nrChannels, 0);

	glEnable(GL_TEXTURE_2D);
	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);
	// sample: specify texture parameters
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	// set the active texture
	if (data) {
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
		stbi_image_free(data);
	}
	else {
		std::cout << "Failed to load Texture" << std::endl;
		glfwTerminate();
	}
}


void AddLight(mlBezier &mlbezier)
{

}

int main()
{
	glfwInit();

	GLFWwindow * window = glfwCreateWindow(SCR_WINDTH, SCR_HEIGHT, "hello", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glewExperimental = GL_TRUE;
	glewInit();



	mlBezier mlbezier;
	mlbezier.divs = 25;
	mlbezier.mlCreateBeizermesh();
	mlbezier.mlTriangularization();



	unsigned int texture = 0;
	initTexture(texture);

	initPMV();
	glewInit();

	//开灯
	GLfloat sun_light_position[] = { 10.0f, 10.0f, 10.0f, 1.0f };
	GLfloat sun_light_ambient[] = { 0.5f, 0.5f, 0.5f, 1.0f };
	GLfloat sun_light_diffuse[] = { 1.0f, 0.5f, 0.5f, 1.0f };
	GLfloat sun_light_specular[] = { 1.0f, 1.0f, 0.0f, 1.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, sun_light_position);
	glLightfv(GL_LIGHT0, GL_AMBIENT, sun_light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, sun_light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, sun_light_specular);
	
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);


	while (!glfwWindowShouldClose(window))
	{
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glPointSize(5);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		drawControlPoint(mlbezier);

		drawBezierSurface(mlbezier);

		processInput(window);
		glfwSetCursorPosCallback(window, mouse_callback);

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glfwTerminate();
	return 0;
}

void processInput(GLFWwindow *window)
{	
	GLfloat unitchange = 0.01f;
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
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