DIRS=nanoboot slot_70 slot_71 slot_72 slot_73 slot_A0 slot_A1 slot_C0 slot_D0 slot_F0 slot_F1 slot_EE ../backend

all:
	for dir in $(DIRS) ; do \
		make -C "$$dir" $@;\
	done

clean:
	for dir in $(DIRS) ; do \
		make -C "$$dir" $@;\
	done
	- rm /tmp/data_exchange/nanoslot
