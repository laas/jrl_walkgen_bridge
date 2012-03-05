#ifndef JRL_WALKGEN_BRIDGE_MORISAWA_HH
# define JRL_WALKGEN_BRIDGE_MORISAWA_HH
# include <utility>

# include <jrl/mal/matrixabstractlayer.hh>
# include <jrl/walkgen/patterngeneratorinterface.hh>

# include <walk_interfaces/pattern-generator.hh>

namespace jrlWalkgenBridge
{
  class Morisawa2007 : public walk::PatternGenerator2d
  {
  public:
    typedef boost::shared_ptr<PatternGeneratorJRL::PatternGeneratorInterface>
    patternGeneratorInterfacePtr_t;
    typedef boost::shared_ptr<CjrlHumanoidDynamicRobot>
    CjrlHumanoidDynamicRobotPtr_t;

    explicit Morisawa2007(const std::string& robotDescription);
    explicit Morisawa2007(const Morisawa2007&);
    ~Morisawa2007();

    Morisawa2007& operator= (const Morisawa2007&);
  protected:
    virtual void computeTrajectories();
  private:
    CjrlHumanoidDynamicRobotPtr_t robot_;
    patternGeneratorInterfacePtr_t pgi_;
  };
} // end of namespace jrlWalkgenBridge.

#endif //! JRL_WALKGEN_BRIDGE_MORISAWA_HH
