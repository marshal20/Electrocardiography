#include "axis_renderer.h"
#include "main_dev.h"
#include "opengl/gl_texture.h"
#include "opengl/gl_framebuffer.h"
#include "model.h"
#include "math.h"
#include "transform.h"
#include "renderer3d.h"

static Mesh create_xarrow_mesh();

AxisRenderer::AxisRenderer(int width, int height)
{
	m_fr = new ForwardRenderer();
	
	glTexture* color_tex = glTexture::create(width, height, Format::FORMAT_RGBA, Type::TYPE_UNSIGNED_BYTE);
	glTexture* depth_tex = glTexture::create(width, height, Format::FORMAT_DEPTH, Type::TYPE_FLOAT);
	m_frame_buffer = glFrameBuffer::create({ color_tex }, depth_tex);

	m_xarrow_mesh = create_xarrow_mesh();

	m_width = width;
	m_height = height;
}

AxisRenderer::~AxisRenderer()
{
	delete m_fr;
	delete m_frame_buffer;
	freeMesh(m_xarrow_mesh);
}

void AxisRenderer::render(const LookAtCamera& camera)
{
	glGraphicsDevice* gldev = gdevGet();

	// setup camera
	LookAtCamera new_camera = camera;
	new_camera.look_at = glm::vec3(0, 0, 0);
	new_camera.eye = glm::normalize(camera.eye - camera.look_at) * glm::vec3(1.8);
	new_camera.fov = 90*PI/180;
	new_camera.aspect = ((float)m_width)/((float)m_height);

	// bind framebuffer
	m_frame_buffer->bind();

	// clear color buffer with transparent background
	gldev->clearColorBuffer(0, 0, 0, 0);
	// enable depth testing
	gldev->depthTest(STATE_ENABLED);
	gldev->depthFunc(COMPARISON_LESS);
	gldev->clearDepthBuffer(1.0);
	
	// TODO: use arrows insteat of lines
	//// render axis
	//m_fr->setViewProjectionMatrix(new_camera.calculateViewProjection());
	//// x-axis
	//m_xarrow_mesh.material.diffuse_color = glm::vec3(1, 0, 0);
	//m_fr->renderMesh(rotateEulerXYZ(0, 0, 0), &m_xarrow_mesh);
	//// y-axis
	//m_xarrow_mesh.material.diffuse_color = glm::vec3(0, 1, 0);
	//m_fr->renderMesh(rotateEulerXYZ(0, 0, 90*PI/180), &m_xarrow_mesh);
	//// z-axis
	//m_xarrow_mesh.material.diffuse_color = glm::vec3(0, 0, 1);
	//m_fr->renderMesh(rotateEulerXYZ(0, -90*PI/180, 0), &m_xarrow_mesh);

	// render axis
	Renderer3D::setProjection(new_camera.calculateViewProjection());
	// x-axis (red)
	Renderer3D::setStyle(Renderer3D::Style(true, 2, { 1, 0, 0, 1 }, true, { 0.75, 0, 0 ,1 }));
	Renderer3D::drawLine({ 0, 0, 0 }, { 1, 0, 0 });
	// y-axis (green)
	Renderer3D::setStyle(Renderer3D::Style(true, 2, { 0, 1, 0, 1 }, true, { 0.75, 0, 0 ,1 }));
	Renderer3D::drawLine({ 0, 0, 0 }, { 0, 1, 0 });
	// z-axis (blue)
	Renderer3D::setStyle(Renderer3D::Style(true, 2, { 0, 0, 1, 1 }, true, { 0.75, 0, 0 ,1 }));
	Renderer3D::drawLine({ 0, 0, 0 }, { 0, 0, 1 });

	// unbind framebuffer
	m_frame_buffer->unbind();
	gldev->bindBackbuffer();
}

glTexture* AxisRenderer::get_texture()
{
	return m_frame_buffer->getColorTexture(0);
}


static Mesh create_xarrow_mesh()
{
	//Mesh xarrow = loadModelFile("models/xaxis_arrow.fbx").mesh_list[0];
	//xarrow.material.shininess = 0;
	//xarrow.material.specular = 0;
	//xarrow.material.ambient = 1;
	//return xarrow;

	return Mesh();
}

