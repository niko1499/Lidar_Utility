#include <nodelet/nodelet.h>
#include <string>
namespace my_nodelet
{

    class MyNodeletClass : public nodelet::Nodelet
    {
        public:
            virtual void onInit();
    };

}
