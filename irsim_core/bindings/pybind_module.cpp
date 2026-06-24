#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include "world.h"
#include "astar.h"
#include <cstdint>
#include <cstring>
#include <algorithm>

namespace py = pybind11;

// Helper: convert Python list/tuple of floats to Obstacle
// verts_storage: optional vector to store polygon vertices (caller must keep alive)
static Obstacle py_to_obstacle(const py::dict& d,
                               std::vector<Vec2>* verts_storage = nullptr) {
    Obstacle obs;
    std::string type = d["type"].cast<std::string>();
    if (type == "circle") {
        obs.type = ShapeType::CIRCLE;
        obs.center = {d["x"].cast<float>(), d["y"].cast<float>()};
        obs.radius = d["radius"].cast<float>();
    } else if (type == "rect") {
        obs.type = ShapeType::RECT;
        obs.center = {d["x"].cast<float>(), d["y"].cast<float>()};
        obs.half_w = d["half_w"].cast<float>();
        obs.half_h = d["half_h"].cast<float>();
    } else if (type == "polygon") {
        obs.type = ShapeType::POLYGON;
        auto vlist = d["vertices"].cast<py::list>();
        obs.n_verts = (int)vlist.size();
        if (verts_storage && obs.n_verts >= 3) {
            verts_storage->clear();
            verts_storage->reserve(obs.n_verts);
            for (auto item : vlist) {
                auto pt = item.cast<py::list>();
                verts_storage->push_back({pt[0].cast<float>(), pt[1].cast<float>()});
            }
            obs.verts = verts_storage->data();
            obs.compute_aabb();
        } else {
            obs.verts = nullptr;
        }
    }
    return obs;
}

PYBIND11_MODULE(_core, m) {
    m.doc() = "IR-SIM C++ core: accelerated LiDAR, collision, kinematics, A*";

    // ── KinematicsType enum ───────────────────────────────────
    py::enum_<KinematicsType>(m, "KinematicsType")
        .value("DIFF", KinematicsType::DIFF)
        .value("OMNI", KinematicsType::OMNI)
        .value("ACKER", KinematicsType::ACKER)
        .value("OMNI_ANGULAR", KinematicsType::OMNI_ANGULAR)
        .export_values();

    // ── SimWorld ──────────────────────────────────────────────
    py::class_<SimWorld>(m, "SimWorld")
        .def(py::init<>())
        .def("set_step_time", &SimWorld::set_step_time)
        .def("step_time", &SimWorld::step_time)
        .def("add_robot", [](SimWorld& w, int kin, float x, float y, float theta,
                             py::array_t<float> vel_min = py::array_t<float>(),
                             py::array_t<float> vel_max = py::array_t<float>(),
                             py::array_t<float> vel_acc = py::array_t<float>()) -> int {
            float vmin[3] = {-1.0f, -1.0f, -1.0f};
            float vmax[3] = { 1.0f,  1.0f,  1.0f};
            float vacc[3] = { 1.0f,  1.0f,  1.0f};
            if (vel_min.size() > 0) { auto b = vel_min.request();
                for (size_t i = 0; i < size_t(b.size) && i < 3; i++)
                    vmin[i] = static_cast<const float*>(b.ptr)[i]; }
            if (vel_max.size() > 0) { auto b = vel_max.request();
                for (size_t i = 0; i < size_t(b.size) && i < 3; i++)
                    vmax[i] = static_cast<const float*>(b.ptr)[i]; }
            if (vel_acc.size() > 0) { auto b = vel_acc.request();
                for (size_t i = 0; i < size_t(b.size) && i < 3; i++)
                    vacc[i] = static_cast<const float*>(b.ptr)[i]; }
            return w.add_robot(static_cast<KinematicsType>(kin), x, y, theta, vmin, vmax, vacc);
        }, py::arg("kinematics"), py::arg("x"), py::arg("y"), py::arg("theta"),
           py::arg("vel_min") = py::array_t<float>(),
           py::arg("vel_max") = py::array_t<float>(),
           py::arg("vel_acc") = py::array_t<float>())
        .def("set_robot_vertices", [](SimWorld& w, int id, py::array_t<float> verts) {
            auto buf = verts.request();
            int n = (int)buf.size / 2;
            auto ptr = static_cast<const float*>(buf.ptr);
            std::vector<Vec2> v(n);
            for (int i = 0; i < n; i++) v[i] = {ptr[i*2], ptr[i*2+1]};
            w.set_robot_vertices(id, v.data(), n);
        })
        .def("add_obstacle", [](SimWorld& w, py::dict obs_dict) -> int {
            std::string type = obs_dict["type"].cast<std::string>();
            Obstacle obs;
            if (type == "circle") {
                obs.type = ShapeType::CIRCLE;
                obs.center = {obs_dict["x"].cast<float>(), obs_dict["y"].cast<float>()};
                obs.radius = obs_dict["radius"].cast<float>();
                return w.add_obstacle(obs);
            } else if (type == "rect") {
                obs.type = ShapeType::RECT;
                obs.center = {obs_dict["x"].cast<float>(), obs_dict["y"].cast<float>()};
                obs.half_w = obs_dict["half_w"].cast<float>();
                obs.half_h = obs_dict["half_h"].cast<float>();
                return w.add_obstacle(obs);
            } else if (type == "polygon") {
                auto vlist = obs_dict["vertices"].cast<py::list>();
                std::vector<Vec2> verts;
                for (auto item : vlist) {
                    auto pt = item.cast<py::list>();
                    verts.push_back({pt[0].cast<float>(), pt[1].cast<float>()});
                }
                return w.add_polygon_obstacle(verts);
            }
            return -1;
        })
        .def("step", [](SimWorld& w, py::array_t<float> actions, int action_dim) {
            auto buf = actions.request();
            w.step(static_cast<const float*>(buf.ptr), action_dim);
        })
        .def("raycast", [](SimWorld& w, int robot_id,
                           py::array_t<float> angles, float range_max) -> py::array_t<float>
        {
            auto buf = angles.request();
            int n = (int)buf.size;
            auto result = py::array_t<float>(n);
            auto res_buf = result.request();
            w.raycast(robot_id,
                      static_cast<const float*>(buf.ptr), n, range_max,
                      static_cast<float*>(res_buf.ptr));
            return result;
        })
        .def("check_robot_collision", &SimWorld::check_robot_collision)
        .def("num_robots", &SimWorld::num_robots)
        .def("num_obstacles", &SimWorld::num_obstacles)
        .def("get_robot_pose", [](SimWorld& w, int id) -> py::tuple {
            const auto& r = w.robot(id);
            return py::make_tuple(r.x, r.y, r.theta);
        })
        .def("get_robot_velocity", [](SimWorld& w, int id) -> py::tuple {
            const auto& r = w.robot(id);
            return py::make_tuple(r.vx, r.vy, r.omega);
        })
        .def("get_robot_collision", &SimWorld::check_robot_collision)
        .def("astar", [](SimWorld& w) -> AStarPlanner& { return w.astar(); },
             py::return_value_policy::reference);

    // ── AStarPlanner ──────────────────────────────────────────
    py::class_<AStarPlanner>(m, "AStarPlanner")
        .def(py::init<>())
        .def("set_grid", [](AStarPlanner& ap, py::array_t<uint8_t> grid, int w, int h, float res) {
            auto buf = grid.request();
            ap.set_grid(static_cast<const uint8_t*>(buf.ptr), w, h, res);
        })
        .def("plan", [](AStarPlanner& ap, float sx, float sy, float gx, float gy) -> py::array_t<float> {
            auto path = ap.plan(sx, sy, gx, gy);
            auto result = py::array_t<float>(path.size());
            if (!path.empty()) {
                auto buf = result.request();
                std::memcpy(buf.ptr, path.data(), path.size() * sizeof(float));
            }
            return result;
        });

    // ── Standalone utility functions ──────────────────────────
    m.def("lidar_raycast_scalar", [](float ox, float oy, float heading,
                                      py::array_t<float> angles, float range_max,
                                      py::list obstacles_py) -> py::array_t<float>
    {
        auto buf = angles.request();
        int n = (int)buf.size;
        auto angles_ptr = static_cast<const float*>(buf.ptr);

        // Parse obstacles from Python list
        std::vector<Obstacle> obs_list;
        std::vector<Vec2> verts_buf;  // persistent storage for polygon vertices
        for (auto item : obstacles_py) {
            auto d = item.cast<py::dict>();
            obs_list.push_back(py_to_obstacle(d, &verts_buf));
        }

        auto result = py::array_t<float>(n);
        auto res_buf = result.request();
        lidar_raycast_scalar({ox, oy}, heading, angles_ptr, n, range_max,
                             obs_list.data(), (int)obs_list.size(),
                             static_cast<float*>(res_buf.ptr));
        return result;
    }, "Standalone LiDAR raycast (scalar)");

    m.def("lidar_raycast", [](float ox, float oy, float heading,
                               py::array_t<float> angles, float range_max,
                               py::list obstacles_py) -> py::array_t<float>
    {
        auto buf = angles.request();
        int n = (int)buf.size;
        auto angles_ptr = static_cast<const float*>(buf.ptr);

        std::vector<Obstacle> obs_list;
        std::vector<Vec2> verts_buf;
        for (auto item : obstacles_py) {
            auto d = item.cast<py::dict>();
            obs_list.push_back(py_to_obstacle(d, &verts_buf));
        }

        auto result = py::array_t<float>(n);
        auto res_buf = result.request();
        lidar_raycast({ox, oy}, heading, angles_ptr, n, range_max,
                      obs_list.data(), (int)obs_list.size(),
                      static_cast<float*>(res_buf.ptr));
        return result;
    }, "Auto-selected LiDAR raycast");

    // ── Kinematics standalone ─────────────────────────────────
    m.def("step_diff", [](float x, float y, float theta,
                           float v, float omega, float dt) -> py::tuple {
        step_diff(x, y, theta, v, omega, dt);
        return py::make_tuple(x, y, theta);
    });
    m.def("step_omni", [](float x, float y, float theta,
                           float vx, float vy, float dt) -> py::tuple {
        step_omni(x, y, theta, vx, vy, dt);
        return py::make_tuple(x, y, theta);
    });
}
