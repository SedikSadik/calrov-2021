
! h264parse
! queue
! rtph264pay config-interval=10 pt=96
! multiudpsink clients=192.168.2.1:5600, 192.168.2.1:4777

        GSTREAMER ÇOKLU İZLEME AYARLARI

! h264parse
! queue
! rtph264pay config-interval=10 pt=96
! udpsink host=192.168.2.1 port=5600

        GSTREAMER VARSAYILAN AYARLARI



        