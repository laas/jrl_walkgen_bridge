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
    explicit Kajita2003();
    explicit Kajita2003(const Kajita2003&);
    ~Kajita2003();

    Kajita2003& operator= (const Kajita2003&);
  protected:
    virtual void computeTrajectories();
  private:
    PatternGeneratorJRL::PatternGeneratorInterface pgi_;
  };
} // end of namespace jrlWalkgenBridge.

#endif //! JRL_WALKGEN_BRIDGE
