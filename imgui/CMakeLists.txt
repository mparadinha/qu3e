set(imgui_srcs
	imgui.cpp
	imgui_demo.cpp
	imgui_draw.cpp
	imgui_tables.cpp
	imgui_widgets.cpp
	backends/imgui_impl_glfw.cpp
	backends/imgui_impl_opengl3.cpp
)

set(imgui_hdrs
	imgui.h
	imgui_internal.h
	imstb_rectpack.h
	imstb_textedit.h
	imstb_truetype.h
	backends/imgui_impl_glfw.h
	backends/imgui_impl_opengl3.h
	backends/imgui_impl_opengl3_loader.h
)

include_directories (
	${OPENGL_INCLUDE_DIR}
	.
)

add_library(imgui
	${imgui_srcs}
	${imgui_hdrs}
)
