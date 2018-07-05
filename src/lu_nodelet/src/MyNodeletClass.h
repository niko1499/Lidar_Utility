#include <nodelet/nodelet.h>

namespace my_nodelet
{

    class MyNodeletClass : public nodelet::Nodelet
    {
        public:
            virtual void onInit();
    };

}
