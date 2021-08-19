#include "clouds.h"

// Needed to generate stb_image binaries. Should only define in exactly one source file importing stb_image.h.
#define STB_IMAGE_IMPLEMENTATION
#include "misc/stb_image.h"

using namespace nanogui;
using namespace std;

Vector3D load_texture(int frame_idx, GLuint handle, const char* where) {
  Vector3D size_retval;
  
  if (strlen(where) == 0) return size_retval;
  
  glActiveTexture(GL_TEXTURE0 + frame_idx);
  glBindTexture(GL_TEXTURE_2D, handle);
  
  int img_x, img_y, img_n;
  unsigned char* img_data = stbi_load(where, &img_x, &img_y, &img_n, 3);
  size_retval.x = img_x;
  size_retval.y = img_y;
  size_retval.z = img_n;
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_x, img_y, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
  stbi_image_free(img_data);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  
  return size_retval;
}

void load_cubemap(int frame_idx, GLuint handle, const std::vector<std::string>& file_locs) {
  glActiveTexture(GL_TEXTURE0 + frame_idx);
  glBindTexture(GL_TEXTURE_CUBE_MAP, handle);
  for (int side_idx = 0; side_idx < 6; ++side_idx) {
    int img_x, img_y, img_n;
    unsigned char* img_data = stbi_load(file_locs[side_idx].c_str(), &img_x, &img_y, &img_n, 3);
    glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + side_idx, 0, GL_RGB, img_x, img_y, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
    stbi_image_free(img_data);
    std::cout << "Side " << side_idx << " has dimensions " << img_x << ", " << img_y << std::endl;

    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
  }
}

void Clouds::load_textures() {
  glGenTextures(1, &m_gl_texture_1);
  glGenTextures(1, &m_gl_texture_2);
  glGenTextures(1, &m_gl_texture_3);
  glGenTextures(1, &m_gl_texture_4);
  glGenTextures(1, &m_gl_cubemap_tex);
  
  m_gl_texture_1_size = load_texture(1, m_gl_texture_1, (m_project_root + "/textures/texture_1.png").c_str());
  m_gl_texture_2_size = load_texture(2, m_gl_texture_2, (m_project_root + "/textures/texture_2.png").c_str());
  m_gl_texture_3_size = load_texture(3, m_gl_texture_3, (m_project_root + "/textures/texture_3.png").c_str());
  m_gl_texture_4_size = load_texture(4, m_gl_texture_4, (m_project_root + "/textures/texture_4.png").c_str());
  
  std::cout << "Texture 1 loaded with size: " << m_gl_texture_1_size << std::endl;
  std::cout << "Texture 2 loaded with size: " << m_gl_texture_2_size << std::endl;
  std::cout << "Texture 3 loaded with size: " << m_gl_texture_3_size << std::endl;
  std::cout << "Texture 4 loaded with size: " << m_gl_texture_4_size << std::endl;
  
  std::vector<std::string> cubemap_fnames = {
    m_project_root + "/textures/cube/posx.jpg",
    m_project_root + "/textures/cube/negx.jpg",
    m_project_root + "/textures/cube/posy.jpg",
    m_project_root + "/textures/cube/negy.jpg",
    m_project_root + "/textures/cube/posz.jpg",
    m_project_root + "/textures/cube/negz.jpg"
  };
  
  load_cubemap(5, m_gl_cubemap_tex, cubemap_fnames);
  std::cout << "Loaded cubemap texture" << std::endl;
}

void Clouds::load_shaders() {
  std::set<std::string> shader_folder_contents;
  bool success = FileUtils::list_files_in_directory(m_project_root + "/shaders", shader_folder_contents);
  if (!success) {
    std::cout << "Error: Could not find the shaders folder!" << std::endl;
  }
  
  std::string std_vert_shader = m_project_root + "/shaders/Default.vert";
  
  for (const std::string& shader_fname : shader_folder_contents) {
    std::string file_extension;
    std::string shader_name;
    
    FileUtils::split_filename(shader_fname, shader_name, file_extension);
    
    if (file_extension != "frag") {
      std::cout << "Skipping non-shader file: " << shader_fname << std::endl;
      continue;
    }
    
    std::cout << "Found shader file: " << shader_fname << std::endl;
    
    // Check if there is a proper .vert shader or not for it
    std::string vert_shader = std_vert_shader;
    std::string associated_vert_shader_path = m_project_root + "/shaders/" + shader_name + ".vert";
    if (FileUtils::file_exists(associated_vert_shader_path)) {
      vert_shader = associated_vert_shader_path;
    }
    
    std::shared_ptr<GLShader> nanogui_shader = make_shared<GLShader>();
    nanogui_shader->initFromFiles(shader_name, vert_shader,
                                  m_project_root + "/shaders/" + shader_fname);
    
    // Special filenames are treated a bit differently
    ShaderTypeHint hint;
    if (shader_name == "Wireframe") {
      hint = ShaderTypeHint::WIREFRAME;
      std::cout << "Type: Wireframe" << std::endl;
    }
    
    UserShader user_shader(shader_name, nanogui_shader, hint);
    
    shader_map.emplace( shader_name, user_shader );
    // shader_map[shader_name] = user_shader;
    shaders.push_back(user_shader);
    shaders_combobox_names.push_back(shader_name);
  }
  
  // Assuming that it's there, use "Wireframe" by default
  for (size_t i = 0; i < shaders_combobox_names.size(); ++i) {
    if (shaders_combobox_names[i] == "Wireframe") {
      active_shader_idx = i;
      break;
    }
  }
}

Clouds::Clouds(std::string project_root, Screen *screen)
: m_project_root(project_root) {
  this->screen = screen;
  
  this->load_shaders();
  this->load_textures();

  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_DEPTH_TEST);

}

Clouds::~Clouds() {
  for (auto shader : shaders) {
    shader.nanogui_shader->free();
  }

  glDeleteTextures(1, &m_gl_texture_1);
  glDeleteTextures(1, &m_gl_texture_2);
  glDeleteTextures(1, &m_gl_texture_3);
  glDeleteTextures(1, &m_gl_texture_4);
  glDeleteTextures(1, &m_gl_cubemap_tex);
}

int Clouds::getFPS() {
  return frames_per_sec;
}

/**
 * Initializes the clouds simulation and spawns a new thread to separate
 * rendering from simulation.
 */
void Clouds::init() {
  // Initialize GUI
  screen->setSize(default_window_size);
  initGUI(screen);

  // Initialize camera

  CGL::Collada::CameraInfo camera_info;
  camera_info.hFov = 50;
  camera_info.vFov = 35;
  camera_info.nClip = 0.01;
  camera_info.fClip = 10000;

  // Try to intelligently figure out the camera target
  CGL::Vector3D target( 250, -20, 100 );
  CGL::Vector3D c_dir( 0, 0, 0 );
  canonical_view_distance = 2;
  scroll_rate = canonical_view_distance / 10;

  view_distance = canonical_view_distance * 2;
  min_view_distance = canonical_view_distance / 10.0;
  max_view_distance = canonical_view_distance * 200.0;

  // canonicalCamera is a copy used for view resets

  camera.place(target, acos(c_dir.y), atan2(c_dir.x, c_dir.z), view_distance,
               min_view_distance, max_view_distance);
  canonicalCamera.place(target, acos(c_dir.y), atan2(c_dir.x, c_dir.z),
                        view_distance, min_view_distance, max_view_distance);

  screen_w = default_window_size(0);
  screen_h = default_window_size(1);

  camera.configure(camera_info, screen_w, screen_h);
  canonicalCamera.configure(camera_info, screen_w, screen_h);

  // only generate texture once
  glGenTextures( 1, &density_tex_id );

  generateBoundingBox();
  generatePoints();

  noise_packed = new std::vector<unsigned char>( texture_pixels * texture_pixels * texture_pixels * 4, 0 );
  std::cout << "Generating Perlin Noise...\n";
  generatePerlinNoise3DTexture( texture_pixels );
  std::cout << "Generating Worley Noise...\n";
  generateWorleyNoise3DTexture( worley_cells, texture_pixels );
  loadPackedNoiseTexture();

  // Must come after generating density values
  // generateDensityTexture();
}

bool Clouds::isAlive() { return is_alive; }

void Clouds::update( double dt_val ) { cloud_offset.z() += dt_val / 10.f; }

void Clouds::updateGUI( double avgFPS ) {
  // Update GUI
  // fps_box->setValue( std::ceil( avgFPS ) );
}

// ----------------------------------------------------------------------------
// CAMERA CALCULATIONS
//
// On initial load, +z faces camera, +x points right, +y points up
// ----------------------------------------------------------------------------

void Clouds::resetCamera() { camera.copy_placement(canonicalCamera); }

Matrix4f Clouds::getProjectionMatrix() {
  Matrix4f perspective;
  perspective.setZero();

  double cam_near = camera.near_clip();
  double cam_far = camera.far_clip();

  double theta = camera.v_fov() * PI / 360;
  double range = cam_far - cam_near;
  double invtan = 1. / tanf(theta);

  perspective(0, 0) = invtan / camera.aspect_ratio();
  perspective(1, 1) = invtan;
  perspective(2, 2) = -(cam_near + cam_far) / range;
  perspective(3, 2) = -1;
  perspective(2, 3) = -2 * cam_near * cam_far / range;
  perspective(3, 3) = 0;

  return perspective;
}

Matrix4f Clouds::getViewMatrix() {
  Matrix4f lookAt;
  Matrix3f R;

  lookAt.setZero();

  // Convert CGL vectors to Eigen vectors
  // TODO: Find a better way to do this!

  CGL::Vector3D c_pos = camera.position();
  CGL::Vector3D c_udir = camera.up_dir();
  CGL::Vector3D c_target = camera.view_point();

  Vector3f eye(c_pos.x, c_pos.y, c_pos.z);
  Vector3f up(c_udir.x, c_udir.y, c_udir.z);
  Vector3f target(c_target.x, c_target.y, c_target.z);

  R.col(2) = (eye - target).normalized();
  R.col(0) = up.cross(R.col(2)).normalized();
  R.col(1) = R.col(2).cross(R.col(0));

  lookAt.topLeftCorner<3, 3>() = R.transpose();
  lookAt.topRightCorner<3, 1>() = -R.transpose() * eye;
  lookAt(3, 3) = 1.0f;

  return lookAt;
}

// ----------------------------------------------------------------------------
// EVENT HANDLING
// ----------------------------------------------------------------------------

bool Clouds::cursorPosCallbackEvent(double x, double y) {
  if (left_down && !middle_down && !right_down) {
    if (ctrl_down) {
      mouseRightDragged(x, y);
    } else {
      mouseLeftDragged(x, y);
    }
  } else if (!left_down && !middle_down && right_down) {
    mouseRightDragged(x, y);
  } else if (!left_down && !middle_down && !right_down) {
    mouseMoved(x, y);
  }

  mouse_x = x;
  mouse_y = y;

  return true;
}

bool Clouds::mouseButtonCallbackEvent(int button, int action, int modifiers) {
  switch (action) {
  case GLFW_PRESS:
    switch (button) {
    case GLFW_MOUSE_BUTTON_LEFT:
      left_down = true;
      break;
    case GLFW_MOUSE_BUTTON_MIDDLE:
      middle_down = true;
      break;
    case GLFW_MOUSE_BUTTON_RIGHT:
      right_down = true;
      break;
    }
    return true;

  case GLFW_RELEASE:
    switch (button) {
    case GLFW_MOUSE_BUTTON_LEFT:
      left_down = false;
      break;
    case GLFW_MOUSE_BUTTON_MIDDLE:
      middle_down = false;
      break;
    case GLFW_MOUSE_BUTTON_RIGHT:
      right_down = false;
      break;
    }
    return true;
  }

  return false;
}

void Clouds::mouseMoved(double x, double y) { y = screen_h - y; }

void Clouds::mouseLeftDragged(double x, double y) {
  float dx = x - mouse_x;
  float dy = y - mouse_y;

  camera.rotate_by(-dy * (PI / screen_h), -dx * (PI / screen_w));
}

void Clouds::mouseRightDragged(double x, double y) {
  camera.move_by(mouse_x - x, y - mouse_y, canonical_view_distance);
}

bool Clouds::keyCallbackEvent(int key, int scancode, int action,
                                      int mods) {
  ctrl_down = (bool)(mods & GLFW_MOD_CONTROL);

  if (action == GLFW_PRESS) {
    switch (key) {
    case GLFW_KEY_ESCAPE:
      is_alive = false;
      break;
    case ' ':
      resetCamera();
      break;
    case 'p':
    case 'P':
      is_paused = !is_paused;
      break;
    case 'n':
    case 'N':
      if (is_paused) {
        is_paused = false;
        drawContents();
        is_paused = true;
      }
      break;
    }
  }

  return true;
}

bool Clouds::dropCallbackEvent(int count, const char **filenames) {
  return true;
}

bool Clouds::scrollCallbackEvent(double x, double y) {
  camera.move_forward(y * scroll_rate);
  return true;
}

bool Clouds::resizeCallbackEvent(int width, int height) {
  screen_w = width;
  screen_h = height;

  camera.set_screen_size(screen_w, screen_h);
  return true;
}

void Clouds::initGUI(Screen *screen) {
  Window *window;
  
  window = new Window(screen, "Cloud Simulation");
  window->setPosition(Vector2i( 15, 15 ));
  window->setLayout(new GroupLayout(15, 6, 14, 5));

  /*
  new Label(window, "Buttons", "sans-bold");
  {
    Button *b1 = new Button(window, "Bounding Points");
    b1->setFlags(Button::ToggleButton);
    b1->setPushed( enableBoundingPointsDraw );
    b1->setFontSize(14);
    b1->setChangeCallback( [&](bool state) { enableBoundingPointsDraw = state; } );

    Button *b2 = new Button(window, "Worley Points");
    b2->setFlags(Button::ToggleButton);
    b2->setPushed( enableWorleyDraw );
    b2->setFontSize(14);
    b2->setChangeCallback( [this](bool state) { enableWorleyDraw = state; } );

    Button *b3 = new Button(window, "Density Points");
    b3->setFlags(Button::ToggleButton);
    b3->setPushed( enableDensityDraw );
    b3->setFontSize(14);
    b3->setChangeCallback( [this](bool state) { enableDensityDraw = state; } );

    Button *b4 = new Button(window, "Lines");
    b4->setFlags(Button::ToggleButton);
    b4->setPushed( enableLinesDraw );
    b4->setFontSize(14);
    b4->setChangeCallback( [this](bool state) { enableLinesDraw = state; } );
  }
  */

  //new Label(window, "Light Properties", "sans-bold");
  {
    Widget *panel = new Widget(window);
    GridLayout *layout =
        new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
    layout->setSpacing(0, 10);
    panel->setLayout(layout);

      new Label(window, "Light Properties", "sans-bold");
      PopupButton *popupBtn = new PopupButton(window, "Popup", ENTYPO_ICON_EXPORT);
      Popup *popup = popupBtn->popup();
      popup->setLayout(new GroupLayout());
      new Label(popup, "Absorption (Sun) :", "sans-bold");
      auto fb1 = new FloatBox<float>(popup);
      fb1->setEditable(true);
      fb1->setFixedSize(Vector2i(100, 20));
      fb1->setFontSize(14);
      fb1->setValue( lt_abs_sun );
      fb1->setSpinnable(true);
      fb1->setCallback([this](float value) { lt_abs_sun = value; });
      // popup right
      new Label(popup, "Absorption (Cloud) :", "sans-bold");
      auto fb2 = new FloatBox<float>(popup);
      fb2->setEditable(true);
      fb2->setFixedSize(Vector2i(100, 20));
      fb2->setFontSize(14);
      fb2->setValue( lt_abs_cloud );
      fb2->setSpinnable(true);
      fb2->setCallback([this](float value) { lt_abs_cloud = value; });

      new Label(popup, "Darkness :", "sans-bold");

      auto fb3 = new FloatBox<float>(popup);
      fb3->setEditable(true);
      fb3->setFixedSize(Vector2i(100, 20));
      fb3->setFontSize(14);
      fb3->setValue( lt_darkness );
      fb3->setSpinnable(true);
      fb3->setCallback([this](float value) { lt_darkness = value; });


    //new Label(panel, "Absorption (Sun) :", "sans-bold");

    //auto fb1 = new FloatBox<float>(panel);


    //new Label(panel, "Absorption (Cloud) :", "sans-bold");





    new Label(popup, "pos x :", "sans-bold");

    auto fb4 = new FloatBox<float>(popup);
    fb4->setEditable(true);
    fb4->setFixedSize(Vector2i(100, 20));
    fb4->setFontSize(14);
    fb4->setValue( pt_light_pos.x() );
    fb4->setSpinnable(true);
    fb4->setCallback([this](float value) { pt_light_pos.x() = value; });

    new Label(popup, "pos y :", "sans-bold");

    auto fb5 = new FloatBox<float>(popup);
    fb5->setEditable(true);
    fb5->setFixedSize(Vector2i(100, 20));
    fb5->setFontSize(14);
    fb5->setValue( pt_light_pos.y() );
    fb5->setSpinnable(true);
    fb5->setCallback([this](float value) { pt_light_pos.y() = value; });

    new Label(popup, "pos z :", "sans-bold");

    auto fb6 = new FloatBox<float>(popup);
    fb6->setEditable(true);
    fb6->setFixedSize(Vector2i(100, 20));
    fb6->setFontSize(14);
    fb6->setValue( pt_light_pos.z() );
    fb6->setSpinnable(true);
    fb6->setCallback([this](float value) { pt_light_pos.z() = value; });

    new Label(popup, "forward scatter :", "sans-bold");

    auto fb7 = new FloatBox<float>(popup);
    fb7->setEditable(true);
    fb7->setFixedSize(Vector2i(100, 20));
    fb7->setFontSize(14);
    fb7->setMinValue( -100 );
    fb7->setMaxValue( 100 );
    fb7->setValue( lt_phase.x() );
    fb7->setSpinnable(true);
    fb7->setCallback([this](float value) { lt_phase.x() = value; });

    new Label(popup, "back scatter :", "sans-bold");

    auto fb8 = new FloatBox<float>(popup);
    fb8->setEditable(true);
    fb8->setFixedSize(Vector2i(100, 20));
    fb8->setFontSize(14);
    fb8->setMinValue( -100 );
    fb8->setMaxValue( 100 );
    fb8->setValue( lt_phase.y() );
    fb8->setSpinnable(true);
    fb8->setCallback([this](float value) { lt_phase.y() = value; });

    new Label(popup, "multiplier :", "sans-bold");

    auto fb9 = new FloatBox<float>(popup);
    fb9->setEditable(true);
    fb9->setFixedSize(Vector2i(100, 20));
    fb9->setFontSize(14);
    fb9->setValue( lt_phase.z() );
    fb9->setSpinnable(true);
    fb9->setCallback([this](float value) { lt_phase.z() = value; });

    new Label(popup, "offset :", "sans-bold");

    auto fb10 = new FloatBox<float>(popup);
    fb10->setEditable(true);
    fb10->setFixedSize(Vector2i(100, 20));
    fb10->setFontSize(14);
    fb10->setValue( lt_phase.w() );
    fb10->setSpinnable(true);
    fb10->setCallback([this](float value) { lt_phase.w() = value; });
  }


  new Label(window, "Parameters", "sans-bold");
  {
    Widget *panel = new Widget(window);
    GridLayout *layout =
        new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
    layout->setSpacing(0, 10);
    panel->setLayout(layout);
    PopupButton *popupBtn = new PopupButton(window, "Popup", ENTYPO_ICON_EXPORT);
    Popup *popup = popupBtn->popup();
    popup->setLayout(new GroupLayout());

    new Label(popup, "num boxes :", "sans-bold");

    num_cells_box = new IntBox<int>(popup);
    num_cells_box->setEditable(true);
    num_cells_box->setFixedSize(Vector2i(100, 20));
    num_cells_box->setFontSize(14);
    num_cells_box->setValue( num_boxes );
    num_cells_box->setMinValue( 0 );
    num_cells_box->setUnits("cells");
    num_cells_box->setSpinnable(true);
    num_cells_box->setCallback([this](int value) { num_boxes = value; });

    new Label(popup, "num cells :", "sans-bold");

    num_cells_box = new IntBox<int>(popup);
    num_cells_box->setEditable(true);
    num_cells_box->setFixedSize(Vector2i(100, 20));
    num_cells_box->setFontSize(14);
    num_cells_box->setValue( num_cells );
    num_cells_box->setMinValue( 0 );
    num_cells_box->setUnits("cells");
    num_cells_box->setSpinnable(true);
    num_cells_box->setCallback([this](int value) {
        num_cells = value;
        generatePoints();
        generateDensityTexture();
    });

    new Label(popup, "pt size :", "sans-bold");

    auto pt_size_box = new FloatBox<float>(popup);
    pt_size_box->setEditable(true);
    pt_size_box->setFixedSize(Vector2i(100, 20));
    pt_size_box->setFontSize(14);
    pt_size_box->setValue( pt_size );
    pt_size_box->setMinValue( 0 );
    pt_size_box->setUnits("px");
    pt_size_box->setSpinnable(true);
    pt_size_box->setCallback([this](float value) {
        pt_size = value;
    });
  }

  new Label(window, "Bounding Box", "sans-bold");
  {
    Widget *panel = new Widget(window);
    GridLayout *layout =
        new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
    layout->setSpacing(0, 10);
    panel->setLayout(layout);
      PopupButton *popupBtn = new PopupButton(window, "Popup", ENTYPO_ICON_EXPORT);
      Popup *popup = popupBtn->popup();
      popup->setLayout(new GroupLayout());
    new Label(popup, "length x :", "sans-bold");
    FloatBox<double> *fb1 = new FloatBox<double>(popup);
    fb1->setEditable(true);
    fb1->setFixedSize(Vector2i(100, 20));
    fb1->setFontSize(14);
    fb1->setValue( bbox_max.x() - bbox_min.x() );
    fb1->setUnits("units");
    fb1->setSpinnable(true);
    fb1->setCallback([this](float value) { 
        bbox_max.x() = value;
        generateBoundingBox();
    });

    new Label(popup, "length y :", "sans-bold");
    FloatBox<double> *fb2 = new FloatBox<double>(popup);
    fb2->setEditable(true);
    fb2->setFixedSize(Vector2i(100, 20));
    fb2->setFontSize(14);
    fb2->setValue( bbox_max.y() - bbox_min.y() );
    fb2->setUnits("units");
    fb2->setSpinnable(true);
    fb2->setCallback([this](float value) { 
        bbox_max.y() = value;
        generateBoundingBox();
    });

    new Label(popup, "length z :", "sans-bold");
    FloatBox<double> *fb3 = new FloatBox<double>(popup);
    fb3->setEditable(true);
    fb3->setFixedSize(Vector2i(100, 20));
    fb3->setFontSize(14);
    fb3->setValue( bbox_max.z() - bbox_min.z() );
    fb3->setUnits("units");
    fb3->setSpinnable(true);
    fb3->setCallback([this](float value) { 
        bbox_max.z() = value;
        generateBoundingBox();
    });
  }

  new Label(window, "Density Sampling", "sans-bold");
  {
    Widget *panel = new Widget(window);
    GridLayout *layout =
        new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
    layout->setSpacing(0, 10);
    panel->setLayout(layout);
      PopupButton *popupBtn = new PopupButton(window, "Popup", ENTYPO_ICON_EXPORT);
      Popup *popup = popupBtn->popup();
      popup->setLayout(new GroupLayout());
    new Label(popup, "Cloud scale :", "sans-bold");
    FloatBox<double> *fb1 = new FloatBox<double>(popup);
    fb1->setEditable(true);
    fb1->setFixedSize(Vector2i(100, 20));
    fb1->setFontSize(14);
    fb1->setValue( cloud_scale );
    fb1->setUnits("units");
    fb1->setSpinnable(true);
    fb1->setCallback([this](float value) { 
        cloud_scale = value;
    });

    new Label(popup, "Density mult :", "sans-bold");
    FloatBox<double> *fb2 = new FloatBox<double>(popup);
    fb2->setEditable(true);
    fb2->setFixedSize(Vector2i(100, 20));
    fb2->setFontSize(14);
    fb2->setValue( density_mult );
    fb2->setUnits("units");
    fb2->setSpinnable(true);
    fb2->setCallback([this](float value) { 
        density_mult = value;
    });

    new Label(popup, "Density thresh :", "sans-bold");
    FloatBox<double> *fb3 = new FloatBox<double>(popup);
    fb3->numberFormat("%2f");
    fb3->setEditable(true);
    fb3->setFixedSize(Vector2i(100, 20));
    fb3->setFontSize(14);
    fb3->setValue( density_thresh );
    fb3->setUnits("units");
    fb3->setSpinnable(true);
    fb3->setCallback([this](float value) { 
        density_thresh = value;
    });

    new Label(popup, "Density samples :", "sans-bold");
    IntBox<int> *fb4 = new IntBox<int>(popup);
    fb4->setEditable(true);
    fb4->setFixedSize(Vector2i(100, 20));
    fb4->setFontSize(14);
    fb4->setValue( density_samples );
    fb4->setMinValue( 1 );
    fb4->setUnits("units");
    fb4->setSpinnable(true);
    fb4->setCallback([this](int value) { 
        density_samples = value;
    });

    new Label(popup, "Offset x :", "sans-bold");
    IntBox<int> *fb5 = new IntBox<int>(popup);
    fb5->setEditable(true);
    fb5->setFixedSize(Vector2i(100, 20));
    fb5->setFontSize(10);
    fb5->setValue( cloud_offset.x() );
    fb5->setUnits("units");
    fb5->setSpinnable(true);
    fb5->setCallback([this](int value) { 
        cloud_offset.x() = value;
    });

    new Label(popup, "Offset y :", "sans-bold");
    IntBox<int> *fb6 = new IntBox<int>(popup);
    fb6->setEditable(true);
    fb6->setFixedSize(Vector2i(100, 20));
    fb6->setFontSize(10);
    fb6->setValue( cloud_offset.y() );
    fb6->setUnits("units");
    fb6->setSpinnable(true);
    fb6->setCallback([this](int value) { 
        cloud_offset.y() = value;
    });

    new Label(popup, "Offset z :", "sans-bold");
    IntBox<int> *fb7 = new IntBox<int>(popup);
    fb7->setEditable(true);
    fb7->setFixedSize(Vector2i(100, 20));
    fb7->setFontSize(10);
    fb7->setValue( cloud_offset.z() );
    fb7->setUnits("units");
    fb7->setSpinnable(true);
    fb7->setCallback([this](int value) { 
        cloud_offset.z() = value;
    });
  }

  /*
  new Label(window, "Sliders", "sans-bold");
  {
    Widget *panel = new Widget(window);
    panel->setLayout(
        new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));

    Slider *s = new Slider(panel);
    s->setValue( slider );
    s->setFixedWidth(105);

    TextBox *percentage = new TextBox(panel);
    percentage->setFixedWidth(75);
    percentage->setValue(to_string( slider ));
    percentage->setUnits("%");
    percentage->setFontSize(14);

    s->setCallback([percentage](float value) {
      percentage->setValue(std::to_string(value));
    });
    s->setFinalCallback([&](float value) {
      slider = value;
    });
  }

  new Label(window, "Statistics", "sans-bold");
  {
    Widget *panel = new Widget(window);
    GridLayout *layout =
        new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
    layout->setSpacing(0, 10);
    panel->setLayout(layout);

    new Label(panel, "AVG FPS :", "sans-bold");

    fps_box = new IntBox<int>(panel);
    fps_box->setEditable(false);
    fps_box->setFixedSize(Vector2i(100, 20));
    fps_box->setFontSize(14);
    fps_box->setValue( 0 );
    fps_box->setUnits("fps");
    fps_box->setSpinnable(false);
  }
  */
}
