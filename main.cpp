#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

	float fRadianAngle = MY_PI*rotation_angle / 180.0f;

	model << cosf(fRadianAngle), -sinf(fRadianAngle), 0, 0,
		sinf(fRadianAngle), cosf(fRadianAngle), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

	float fRadianAngle = MY_PI * eye_fov / 180.0f;
	float fHeight = zFar *tanf(fRadianAngle/2);
	float fWidth = aspect_ratio * fHeight;

	//压缩矩阵
	Eigen::Matrix4f extrusionMatrix = Eigen::Matrix4f::Identity();
	extrusionMatrix << zNear, 0, 0, 0,
		0, zNear, 0, 0,
		0, 0, zFar + zNear, (-1)*zNear*zFar,
		0, 0, 1, 0;

	//正交投影矩阵
	Eigen::Matrix4f orthMatrix = Eigen::Matrix4f::Identity();
	orthMatrix << 2.0 / (2 * fWidth), 0, 0, 0,
		0, 2.0 / (2 * fHeight), 0, 0,
		0, 0,2.0/(zFar-zNear), 0,
		0, 0, 0, 1;

	//映射到屏幕上的矩阵
	Eigen::Matrix4f screenMatrix = Eigen::Matrix4f::Identity();
	screenMatrix << 2*fWidth, 0, 0, fWidth,
		0, 2 * fHeight,  0, fHeight,
		0, 0, 1, (-1)*(zNear+zFar)/2.0f,//屏幕在原点平面
		0, 0, 0, 1;

	projection = extrusionMatrix*orthMatrix*screenMatrix;//先压缩，在正交投影，再平移

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
