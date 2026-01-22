#include "engine/engine.hpp"
#include "engine/defaults.hpp"

#include "app/plugins.hpp"

class App : public Engine {

public:
    App() {
        addPlugin(CollisionPlugin);
        addPlugin(MapLoadPlugin);
        addPlugin(UserPlugin);
        addPlugin(ROSPlusgin);
        addPlugin(VehiclePlugin);
        addPlugin(WorldSettingPlugin);

        addPlugin(CameraPlugin);
        addPlugin(MousePlugin);
        addPlugin(RenderPlugin);
        addPlugin(PhysicsPlugin);
    }
    ~App() {
    }
};

Engine *createApp() {
    return new App();
}