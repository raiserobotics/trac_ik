 /* trac_ik_wrap.i */
 %module trac_ik_wrap

// Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
// Ported to ROS2 by Raise Robotics

 %{
 #include <trac_ik/trac_ik.hpp>
 #include <kdl/frames.hpp>
 #include <urdf/model.h>
 #include <kdl_parser/kdl_parser.hpp>
 #include <limits>
 %}

 %include <std_string.i>
 %include <std_vector.i>

namespace std {
   %template(IntVector) vector<int>;
   %template(DoubleVector) vector<double>;
   %template(StringVector) vector<string>;
   %template(ConstCharVector) vector<const char*>;
}

// Ignore the chain-based constructor (not useful from Python)
%ignore TRAC_IK::TRAC_IK::TRAC_IK(const KDL::Chain& _chain, const KDL::JntArray& _q_min, const KDL::JntArray& _q_max, double _maxtime, double _eps, SolveType _type);
// Ignore the enum-typed constructor — we expose a string-typed version via %extend below
%ignore TRAC_IK::TRAC_IK::TRAC_IK(const std::string& base_link, const std::string& tip_link, const std::string& urdf_string, double _maxtime, double _eps, SolveType _type);
// Private methods; ignored by SWIG automatically, but explicit for clarity.
// The inline definitions at the bottom of trac_ik.hpp appear at namespace
// scope, so SWIG sees both a class-member form and a free-function form.
%ignore TRAC_IK::TRAC_IK::runKDL;
%ignore TRAC_IK::TRAC_IK::runNLOPT;
%ignore TRAC_IK::runKDL;
%ignore TRAC_IK::runNLOPT;
// These are exposed with more Python-friendly signatures below
%ignore TRAC_IK::TRAC_IK::getKDLLimits;
%ignore TRAC_IK::TRAC_IK::setKDLLimits;

%naturalvar;

%include <trac_ik/trac_ik.hpp>

%extend TRAC_IK::TRAC_IK {

    // Constructor that accepts solve_type as a string instead of the C++ enum.
    // Delegates directly to the existing TRAC_IK(string, string, string, ...) constructor.
    TRAC_IK(const std::string& base_link, const std::string& tip_link, const std::string& urdf_string,
      double timeout, double epsilon, const std::string& solve_type="Speed")
    {
        TRAC_IK::SolveType solvetype;
        if (solve_type == "Manipulation1")
            solvetype = TRAC_IK::Manip1;
        else if (solve_type == "Manipulation2")
            solvetype = TRAC_IK::Manip2;
        else if (solve_type == "Distance")
            solvetype = TRAC_IK::Distance;
        else {
            if (solve_type != "Speed")
                fprintf(stderr, "trac_ik: '%s' is not a valid solve_type; defaulting to Speed\n",
                        solve_type.c_str());
            solvetype = TRAC_IK::Speed;
        }
        return new TRAC_IK::TRAC_IK(base_link, tip_link, urdf_string, timeout, epsilon, solvetype);
    }

    // Python-friendly CartToJnt: takes position + quaternion scalars, returns joint vector.
    std::vector<double> CartToJnt(const std::vector<double> q_init,
     const double x, const double y, const double z,
     const double rx, const double ry, const double rz, const double rw,
     const double boundx=0.0, const double boundy=0.0, const double boundz=0.0,
     const double boundrx=0.0, const double boundry=0.0, const double boundrz=0.0)
    {
        KDL::Frame frame(
            KDL::Rotation::Quaternion(rx, ry, rz, rw),
            KDL::Vector(x, y, z)
        );

        KDL::JntArray in(q_init.size()), out(q_init.size());
        for (uint i = 0; i < q_init.size(); i++)
            in(i) = q_init[i];

        KDL::Twist bounds = KDL::Twist::Zero();
        bounds.vel.x(boundx);
        bounds.vel.y(boundy);
        bounds.vel.z(boundz);
        bounds.rot.x(boundrx);
        bounds.rot.y(boundry);
        bounds.rot.z(boundrz);

        int rc = $self->CartToJnt(in, frame, out, bounds);
        std::vector<double> vout;
        if (rc < 0)
            return vout;

        for (uint i = 0; i < q_init.size(); i++)
            vout.push_back(out(i));
        return vout;
    }

    int getNrOfJointsInChain() {
        KDL::Chain chain;
        $self->getKDLChain(chain);
        return (int)chain.getNrOfJoints();
    }

    std::vector<std::string> getJointNamesInChain(const std::string& urdf_string) {
        KDL::Chain chain;
        $self->getKDLChain(chain);
        std::vector<KDL::Segment> chain_segs = chain.segments;
        std::vector<std::string> joint_names;
        urdf::Model robot_model;
        robot_model.initString(urdf_string);
        for (unsigned int i = 0; i < chain_segs.size(); ++i) {
            urdf::JointConstSharedPtr joint =
                robot_model.getJoint(chain_segs[i].getJoint().getName());
            if (joint && joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
                joint_names.push_back(joint->name);
        }
        return joint_names;
    }

    std::vector<std::string> getLinkNamesInChain() {
        KDL::Chain chain;
        $self->getKDLChain(chain);
        std::vector<KDL::Segment> chain_segs = chain.segments;
        std::vector<std::string> link_names;
        for (unsigned int i = 0; i < chain_segs.size(); ++i)
            link_names.push_back(chain_segs[i].getName());
        return link_names;
    }

    std::vector<double> getLowerBoundLimits() {
        KDL::JntArray lb_, ub_;
        $self->getKDLLimits(lb_, ub_);
        std::vector<double> lb;
        for (unsigned int i = 0; i < lb_.rows(); i++)
            lb.push_back(lb_(i));
        return lb;
    }

    std::vector<double> getUpperBoundLimits() {
        KDL::JntArray lb_, ub_;
        $self->getKDLLimits(lb_, ub_);
        std::vector<double> ub;
        for (unsigned int i = 0; i < ub_.rows(); i++)
            ub.push_back(ub_(i));
        return ub;
    }

    void setKDLLimits(const std::vector<double> lb, const std::vector<double> ub) {
        KDL::JntArray lb_, ub_;
        lb_.resize(lb.size());
        for (unsigned int i = 0; i < lb.size(); i++)
            lb_(i) = lb[i];
        ub_.resize(ub.size());
        for (unsigned int i = 0; i < ub.size(); i++)
            ub_(i) = ub[i];
        $self->setKDLLimits(lb_, ub_);
    }

};
