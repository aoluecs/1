//#define _CRT_SECURE_NO_WARNINGS
#include <glad/glad.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "main_ui.h"
#include "YL/voxelization.h"
#include"../Simulation/Simulation/SimulationRender.h"
#include"../Simulation/Simulation/geometry.h"
#include "file.h"



using namespace std;


Drawable  initPath(const vector<vector<glm::vec3>>& v)
{
	shared_ptr<VertexBuffer> vb = make_shared<VertexBuffer>();
	shared_ptr<IndexBuffer> ib = make_shared<IndexBuffer>();
	vector<glm::vec3> vertex;
	vector<unsigned int> index;
	int count = 0;
	for (auto i : v) {
		for (auto j : i) {
			vertex.push_back(j);
			index.push_back(count++);
		}
		index.push_back(~0u);
	}
	vb->setBuffer(vertex);
	ib->setBuffer(index);
	Drawable d(GL_LINE_STRIP, vb, ib);
	return d;
}
int main()
{
	//--------------------------------------------------------------------------------------------------
	if (!glfwInit())   //初始化OpenGl
		return 1;
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	//创建OpenGl窗口
	GLFWwindow* window = glfwCreateWindow(1280, 800, "AM Master CurvedBuilding", NULL, NULL);
	if (window == NULL)
		return 1;

	//设置OpenGl上下文
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1); // Enable vsync

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
		return 1;

	// 设置ImGui上下文.
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;

	//设置颜色风格
	ImGuiStyle* style = &ImGui::GetStyle();
	ImGui::StyleColorsDark();
	//窗口大小
	style->ScaleAllSizes(1.0);
	//字体种类，大小
	io.Fonts->AddFontFromFileTTF("c:/windows/fonts/simhei.ttf", 15.0f, NULL, io.Fonts->GetGlyphRangesChineseFull());
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 330 core");

	//ImVec4 clear_color = ImVec4(0.103f, 0.043f, 0.270f, 1.00f);
	ImVec4 clear_color = ImVec4(1.0f, 1.0f, 1.0f, 1.00f);

	MessagePara message_para;
	MainUI main_ui;
	main_ui.initPara(*window);
	main_ui.initMainConsoleWindow();
	main_ui.initRenderWindow(window);
	PathDrawer* my_path_drawer = nullptr;
	registerEvent(window,1);
	getMessages().displayMessage(u8"初始化完成!", cGreen);


	Shader my_shader("../resource/glsl/light.vs", "../resource/glsl/light.fs", "../resource/glsl/light.gs");
	ShaderManager::get().addShader("../resource/GLSL/light.vs", "../resource/GLSL/light.fs", "../resource/GLSL/light.gs");
	ShaderManager::get().addShader("../resource/GLSL/line.vs", "../resource/GLSL/line.fs");
	ShaderManager::get().addShader("../resource/GLSL/texture.vs", "../resource/GLSL/texture.fs");

	//渲染窗口
	/*CommonView  my_view;
	my_view.attachShader(my_shader);
	my_view.addCutPlane(glm::vec4(0.0, 0.0, -1.0, 100.0));*/
	//CommonRenderWindow  my_render_window(*window);
	STLDrawer* my_drawer = nullptr;

	vector<vector<glm::vec3>>  path;
	vector<vector<glm::vec3>>  normals;
	vector<vector<glm::vec3>>  DKP_pose;

	auto count = 0;
	auto target = 0;
	auto step = 1;
	int path_index = 0;

	bool initFlag = false;
	View* my_view = nullptr;
	glm::vec3 center;
	float scalefactor = 0.1f;

	static float path_r = 2;
	Pipe m_pipe;

	shared_ptr<VertexBuffer> path_buffer = make_shared<VertexBuffer>();
	path_buffer->alloc(sizeof(float) * 3 * 300 * 1000000);
	Drawable tube(GL_TRIANGLES, path_buffer);

	shared_ptr<VertexBuffer> head_buffer = make_shared<VertexBuffer>();
	auto head = readSTL("../曲面路径模拟/直焊枪_Rescaled(0.2).stl");
	//= GenerateCylinder(glm::vec3(0.0, 15.0, 0.0), glm::vec3(0.0, 0.0, 20.0), 3);
	head_buffer->setBuffer(head.points);
	Drawable head_(GL_TRIANGLES, head_buffer);

	shared_ptr<VertexBuffer> DKP_E1_buffer = make_shared<VertexBuffer>();
	auto dkp_e1 = readSTL("../曲面路径模拟/DKP-E1.stl");
	//= GenerateCylinder(glm::vec3(0.0, 15.0, 0.0), glm::vec3(0.0, 0.0, 20.0), 3);
	DKP_E1_buffer->setBuffer(dkp_e1.points);
	Drawable dkp_e1_(GL_TRIANGLES, DKP_E1_buffer);

	shared_ptr<VertexBuffer> DKP_E2_buffer = make_shared<VertexBuffer>();
	auto dkp_e2 = readSTL("../曲面路径模拟/DKP-E2.stl");
	//= GenerateCylinder(glm::vec3(0.0, 15.0, 0.0), glm::vec3(0.0, 0.0, 20.0), 3);
	DKP_E2_buffer->setBuffer(dkp_e2.points);
	Drawable dkp_e2_(GL_TRIANGLES, DKP_E2_buffer);

	shared_ptr<VertexBuffer> DKP_base1_buffer = make_shared<VertexBuffer>();
	auto dkp_base1 = readSTL("../曲面路径模拟/DKP-base-1.stl");
	//= GenerateCylinder(glm::vec3(0.0, 15.0, 0.0), glm::vec3(0.0, 0.0, 20.0), 3);
	DKP_base1_buffer->setBuffer(dkp_base1.points);
	Drawable dkp_base1_(GL_TRIANGLES, DKP_base1_buffer);

	shared_ptr<VertexBuffer> DKP_base2_buffer = make_shared<VertexBuffer>();
	auto dkp_base2 = readSTL("../曲面路径模拟/DKP-base-2.stl");
	//= GenerateCylinder(glm::vec3(0.0, 15.0, 0.0), glm::vec3(0.0, 0.0, 20.0), 3);
	DKP_base2_buffer->setBuffer(dkp_base2.points);
	Drawable dkp_base2_(GL_TRIANGLES, DKP_base2_buffer);

	FrameBufferRender m_frame(0, 0, 1280, 800);
	FrameBufferRender help_frame(0, 0, 1280, 800);
	m_frame.clear(glm::vec4(clear_color.x, clear_color.y, clear_color.z, clear_color.w));
	//help_frame.clear(glm::vec4(clear_color.x, clear_color.y, clear_color.z, clear_color.w));
	my_view = new View(glm::ivec4(0, 0, 1280, 800));
	Light l;
	my_view->addLight(l);
	CoordinateAxis coor;

	ofstream put("file.txt");

	while (!glfwWindowShouldClose(window))
	{
		glfwPollEvents();

		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		//图形渲染
		glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
		glEnable(GL_DEPTH_TEST);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		main_ui.update();
		main_ui.mainConsoleWindow();
		main_ui.showMessageWindow();
		main_ui.showBasicParaWindow();

		//{
		//	vector<glm::vec4>p_rgb(10);
		//	vector<glm::vec3>tempPt(10);
		//	for (int i = 0; i < 10; ++i) {
		//		p_rgb[i] = global_color_arrays[i%5];
		//		tempPt[i] = glm::vec3(i * 1.0, 0.0, 0.0);
		//	}
		//	my_drawer = new STLDrawer;
		//	my_drawer->create();
		//	my_drawer->attach(Vec3(0.5f),
		//		tempPt/*main_ui.voxelModel.curvePoints*/, p_rgb/*main_ui.voxelModel.p_rgb*/);
		//	main_ui.m_render_window_type = 0;
		//}
		
		//stl数据填充
		{
			if (main_ui.is_stl_file_changed) {
				getMessages().displayMessage(u8"STL渲染数据初始化开始...", cGreen);
				if (!main_ui.my_facets.points.empty()) {
					if (my_drawer == nullptr) {
						my_drawer = new STLDrawer;
					}
					my_drawer->create();
					TopologicalFacets& my_facets = main_ui.my_facets;
					my_drawer->attach(&my_facets);
				}
				getMessages().displayMessage(u8"STL渲染数据初始化完成", cGreen);
				main_ui.m_render_window_type = 0;
				main_ui.resetStlFileChangeFlag();
			}

			if (MainUI::voxelFlag) {
				getMessages().displayMessage(u8"体素渲染开始...", cGreen);
				if (!main_ui.voxelModel.voxelPoints.empty()) {
					if (my_drawer == nullptr) {
						my_drawer = new STLDrawer;
					}
					my_drawer->create();
					my_drawer->attach(Vec3(main_ui.p_resolution, main_ui.p_resolution, main_ui.p_resolution),
						main_ui.voxelModel.voxelPoints);
				}
				getMessages().displayMessage(u8"体素渲染完成！", cGreen);
				main_ui.m_render_window_type = 0;
				MainUI::voxelFlag = 0;
				main_ui.voxelModel.voxelPoints.clear();
			}

			if (MainUI::curveFlag) {
				getMessages().displayMessage(u8"曲面渲染开始...", cGreen);
				if (!main_ui.voxelModel.curvePoints.empty()) {
					if (my_drawer == nullptr) {
						my_drawer = new STLDrawer;
					}

					/*vector<glm::vec4>p_rgb(10);
					vector<glm::vec3>tempPt(10);
					for (int i = 0; i < 10; ++i) {
						if (i < 5) {
							p_rgb[i] = global_color_arrays[1];
						}
						else {
							p_rgb[i] = global_color_arrays[2];
						}
						tempPt[i] = main_ui.voxelModel.curvePoints[i];
					}*/

					double voxelSize = main_ui.p_resolution;

					my_drawer->create();
					my_drawer->attach(Vec3(voxelSize, voxelSize, voxelSize),
						main_ui.voxelModel.curvePoints, main_ui.voxelModel.p_rgb);
				}
				getMessages().displayMessage(u8"曲面渲染完成！", cGreen);
				main_ui.m_render_window_type = 0;
				MainUI::curveFlag = 0;
				main_ui.voxelModel.curvePoints.clear();
			}
		}

        //填充数据--路径
		if (my_path_drawer == nullptr) {
			my_path_drawer = new PathDrawer;
		}

		//曲面-路径规划
		if (MainUI::flag)
		{
			my_path_drawer->newLayer();//新建，重置操作
			if (!main_ui.Paths.first.empty()) {
				my_path_drawer->newArea();
				for (int i = 0; i < main_ui.Paths.first.size(); ++i) {
					my_path_drawer->newPath();
					for (int j = 0; j < main_ui.Paths.first[i].size(); ++j) {
						Point3f p(main_ui.Paths.first[i][j].Position_.x, main_ui.Paths.first[i][j].Position_.y, main_ui.Paths.first[i][j].Position_.z);
						my_path_drawer->newVertex(p);
					}
					my_path_drawer->endPath();
				}
				my_path_drawer->end();
				main_ui.m_render_window_type = 1;
			}

			if (!main_ui.Paths.first.empty())
			{
				for (int i = 0; i < main_ui.Paths.first.size(); ++i)
				{
					int num_Path = main_ui.Paths.first[i].size();
					put << num_Path << endl;
					for (int j = 0; j < num_Path; j++)
					{
						put << setw(20) << main_ui.Paths.first[i][j].Position_.x << setw(20) <<
							main_ui.Paths.first[i][j].Position_.y << setw(20) <<
							main_ui.Paths.first[i][j].Position_.z << setw(20) <<
							main_ui.Paths.first[i][j].Proj_.x() << setw(20) <<
							main_ui.Paths.first[i][j].Proj_.y() << setw(20) <<
							main_ui.Paths.first[i][j].Proj_.z() << setw(20) <<
							endl;
					}
				}
				put.close();
			}

			MainUI::flag = 0;
			//main_ui.Paths.first.clear();
			getMessages().displayMessage(u8"任意曲面等间距路径规划渲染完成!", cGreen);
		}

		//生产者-消费者模式
		if(main_ui.voxelModel.is_Finished())
		{
			main_ui.Paths = main_ui.voxelModel.getOriPointPaths();
			my_path_drawer->newLayer();//新建，重置操作
			if (!main_ui.Paths.first.empty()) {
				my_path_drawer->newArea();
				for (int i = 0; i < main_ui.Paths.first.size(); ++i) {
					my_path_drawer->newPath();
					for (int j = 0; j < main_ui.Paths.first[i].size(); ++j) {
						Point3f p(main_ui.Paths.first[i][j].Position_.x, main_ui.Paths.first[i][j].Position_.y, main_ui.Paths.first[i][j].Position_.z);
						my_path_drawer->newVertex(p);
					}
					my_path_drawer->endPath();
				}
				my_path_drawer->end();
				main_ui.m_render_window_type = 1;
			}

			Cubef my_cube = computeCube(path);
			center = glm::vec3(my_cube.midX(), my_cube.midY(), my_cube.midZ());
			scalefactor = 1.5f / (glm::max)((glm::max)(my_cube.w(), my_cube.h()), my_cube.t());

			Drawable path_line = initPath(path);

			main_ui.voxelModel.set_Finished();
			//Paths.clear();
		}

		if (main_ui.voxelModel.all_finish) {
			put.close();
			getMessages().displayMessage(u8"曲面切片-路径规划已完成！", cGreen);//完成之后点击模拟仿真即可
			main_ui.voxelModel.all_finish = false;
		}

		if (main_ui.no_adjust) {
			ImGui::SetNextWindowPos(main_ui.m_window_adjust_para.render_pos);
			ImGui::SetNextWindowSize(main_ui.m_window_adjust_para.render_size);
		}
		{
			auto& my_stl_window = *main_ui.m_stl_render_window;
			auto& my_2d_window = *main_ui.m_path_render_window;

			if (main_ui.no_adjust) {
				ImGui::SetNextWindowPos(main_ui.m_window_adjust_para.render_pos);
				ImGui::SetNextWindowSize(main_ui.m_window_adjust_para.render_size);
			}
			switch (main_ui.m_render_window_type) {
			case 0: {
				registerEvent(window, 0);
				my_stl_window.showWindow();
				if (!main_ui.no_adjust) {
					main_ui.m_window_adjust_para.adjust_x_ratio = my_stl_window.getWindowRatio().x;
					main_ui.m_window_adjust_para.adjust_y_ratio = my_stl_window.getWindowRatio().y;
				}
				if (my_drawer)
					my_stl_window.renderGraphics(*my_drawer);
				break;
			}
			case 1: {
				registerEvent(window, 2);
				my_2d_window.showWindow();
				if (!main_ui.no_adjust) {
					main_ui.m_window_adjust_para.adjust_x_ratio = my_2d_window.getWindowRatio().x;
					main_ui.m_window_adjust_para.adjust_y_ratio = my_2d_window.getWindowRatio().y;
				}
				if (my_path_drawer)
					my_2d_window.renderGraphics(my_path_drawer);
				break;
			}
			default:
				break;
			}
		}
		
		/***********************模拟*****************************************/
		if (main_ui.simulationFlag) {
			if (!initFlag) {
				path.clear();
				normals.clear();
				DKP_pose.clear();
				count = 0;
				target = 0;
				step = 1;
				path_index = 0;

				for (int i = 0; i < main_ui.Paths.first.size(); ++i)
				{
					path.push_back({});
					normals.push_back({});
					DKP_pose.push_back({});
					int num_Path = main_ui.Paths.first[i].size();
					for (int j = 0; j < num_Path; j++)
					{
						path.back().emplace_back(main_ui.Paths.first[i][j].Position_.x,
							main_ui.Paths.first[i][j].Position_.y, main_ui.Paths.first[i][j].Position_.z);
						normals.back().emplace_back(main_ui.Paths.first[i][j].Proj_.x(), main_ui.Paths.first[i][j].Proj_.y(),
							main_ui.Paths.first[i][j].Proj_.z());
						DKP_pose.back().emplace_back(main_ui.Paths.second[i][j].x(), main_ui.Paths.second[i][j].y(),
							main_ui.Paths.second[i][j].z());
					}
				}

				Cubef my_cube = computeCube(path);
				center = glm::vec3(my_cube.midX(), my_cube.midY(), my_cube.midZ());
				scalefactor = 1.5f / (glm::max)((glm::max)(my_cube.w(), my_cube.h()), my_cube.t());

				Drawable path_line = initPath(path);

				m_pipe.Restart(path[0], path_r);

				initFlag = true;
			}

			auto Setup = [&](Shader* s) {
				my_view->setup(s);
			};
			glm::mat4 m_model = glm::mat4(1.0);
			//m_model = glm::rotate(m_model, glm::pi<float>(), glm::vec3(0.0, 1.0, 0.0));
			m_model = glm::scale(m_model, glm::vec3(scalefactor));
			m_model = glm::translate(m_model, -center);
			static int display_w, display_h;
			glfwGetFramebufferSize(window, &display_w, &display_h);

			if (path_index < path.size()) {

				auto& p = path[path_index];
				auto& n = normals[path_index];
				auto& d = DKP_pose[path_index];

				//变位机变换
				auto trans_DKP_before = glm::mat4(1.0f);
				trans_DKP_before = glm::translate(trans_DKP_before, glm::vec3(0, 0, 400));
				auto trans_DKP_After = glm::mat4(1.0f);
				trans_DKP_After = glm::translate(trans_DKP_After, glm::vec3(0, 0, -400));

				//trans_DKP_After* computePosModel(d[count])* trans_DKP_before//变位机变换

				auto triangles = m_pipe.GeneTriangleVerticesWithPath(target, count);
				//auto pos_head = glm::mat4(1.0f);
				auto trans_head = glm::mat4(1.0f);
				trans_head = glm::translate(trans_head, glm::vec3(p[count].x, p[count].y, p[count].z));//喷头平移变换

				m_frame.bind();
				my_view->setViewPort({ 0,0, display_w, display_h });
				glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
				glEnable(GL_DEPTH_TEST);
				glClear(GL_COLOR_BUFFER_BIT);
				glClear(GL_DEPTH_BUFFER_BIT);
				glViewport(0, 0, display_w, display_h);
				glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
				glm::vec4 m_color = { 1.0, 0.0, 0.0, 1.0 };

				tube.getManager()->setShader(0);
				tube.getManager()->setColor(m_color);
				tube.getManager()->setModel(m_model * trans_DKP_After * computePosModel(d[count]) * trans_DKP_before);

				tube.getVertexBuffer()->subData(triangles);

				tube.draw(Setup);

				dkp_e1_.getManager()->setShader(0);
				dkp_e1_.getManager()->setColor(glm::vec4(118 / 255.0, 119 / 255.0, 120 / 255.0, 1.0));
				dkp_e1_.getManager()->setModel(m_model * trans_DKP_After * computePosModel(d[count]) * trans_DKP_before);
				dkp_e1_.draw(Setup);

				dkp_e2_.getManager()->setShader(0);
				dkp_e2_.getManager()->setColor(glm::vec4(118 / 255.0, 119 / 255.0, 120 / 255.0, 1.0));
				dkp_e2_.getManager()->setModel(m_model * trans_DKP_After * computePosModel(d[count])* trans_DKP_before);
				dkp_e2_.draw(Setup);

				dkp_base1_.getManager()->setShader(0);
				dkp_base1_.getManager()->setColor(glm::vec4(118 / 255.0, 119 / 255.0, 120 / 255.0, 1.0));
				dkp_base1_.getManager()->setModel(m_model);
				dkp_base1_.draw(Setup);

				dkp_base2_.getManager()->setShader(0);
				dkp_base2_.getManager()->setColor(glm::vec4(118 / 255.0, 119 / 255.0, 120 / 255.0, 1.0));
				dkp_base2_.getManager()->setModel(m_model * trans_DKP_After * computePosModel(d[count])* trans_DKP_before);
				dkp_base2_.draw(Setup);

				head_.getManager()->setShader(0);
				head_.getManager()->setColor(glm::vec4(0.0, 1.0, 0.0, 1.0));
				head_.getManager()->setModel(m_model * trans_DKP_After * computePosModel(d[count]) * trans_DKP_before * trans_head * computePosModel(n[count]));
				head_.draw(Setup);
				//coor.draw(my_view);

				m_frame.unbind();
				target = count;
				//if (is_play) 
			   // if (ImGui::Button(u8"+"))
				{
					count += step;
				}
				if (count >= p.size() - 1) {
					if (path_index < path.size()) {
						path_index++;
						count = 0;
						target = 0;
						if (path_index < path.size())
							m_pipe.Restart(path[path_index], path_r);
					}
				}

			}
			else {
				m_frame.bind();
				my_view->setViewPort({ 0,0, display_w, display_h });
				glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
				glEnable(GL_DEPTH_TEST);
				glClear(GL_COLOR_BUFFER_BIT);
				glClear(GL_DEPTH_BUFFER_BIT);
				glViewport(0, 0, display_w, display_h);
				glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
				tube.draw(Setup);
				//coor.draw(my_view);
				head_.draw(Setup);
				dkp_e1_.draw(Setup);
				dkp_e2_.draw(Setup);
				dkp_base1_.draw(Setup);
				dkp_base2_.draw(Setup);
				m_frame.unbind();
			}

			{
				glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
				glEnable(GL_DEPTH_TEST);
				glClear(GL_COLOR_BUFFER_BIT);
				glClear(GL_DEPTH_BUFFER_BIT);
				glViewport(0, 0, 1280, 800);
				glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
				m_frame.renderTexture();

				/*glEnable(GL_DEPTH_TEST);
				path_line.getManager()->setShader(1);
				path_line.getManager()->setColor(glm::vec4(0.0, 1.0, 0.0, 1.0));
				path_line.getManager()->setModel(m_model);
				path_line.setRestartPrim(true);
				path_line.draw(Setup);*/
			}
		}
		/***********************模拟*****************************************/


		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		glfwMakeContextCurrent(window);
		glfwSwapBuffers(window);
	}
	// Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}