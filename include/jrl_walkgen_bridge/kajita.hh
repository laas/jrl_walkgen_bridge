#ifndef JRL_WALKGEN_BRIDGE
# define JRL_WALKGEN_BRIDGE
# include <utility>

# include <jrl/mal/matrixabstractlayer.hh>
# include <jrl/walkgen/patterngeneratorinterface.hh>

# include <walk_interfaces/pattern-generator.hh>

namespace jrlWalkgenBridge
{
  class Kajita2003 : public walk::PatternGenerator2d
  {
  public:
    typedef boost::shared_ptr<PatternGeneratorJRL::PatternGeneratorInterface>
    patternGeneratorInterfacePtr_t;
    typedef boost::shared_ptr<CjrlHumanoidDynamicRobot>
    CjrlHumanoidDynamicRobotPtr_t;

    explicit Kajita2003(const std::string& robotDescription);
    explicit Kajita2003(const Kajita2003&);
    ~Kajita2003();

    Kajita2003& operator= (const Kajita2003&);
  protected:
    virtual void computeTrajectories();
  private:
    double step_;
    CjrlHumanoidDynamicRobotPtr_t robot_;
    patternGeneratorInterfacePtr_t pgi_;
  };
} // end of namespace jrlWalkgenBridge.

#endif //! JRL_WALKGEN_BRIDGE
