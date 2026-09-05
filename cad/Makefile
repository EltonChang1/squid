# Render every printable part to STL headlessly and validate bounding boxes.
#   make            -> stl/*.stl
#   make check      -> render + dimension sanity checks
#   make clean
OPENSCAD ?= openscad
OUT      := stl
FLAGS    := --export-format binstl

PARTS := finger_body finger_camera_cap finger_gel_frame rail_base carriage_adapter trigger ft_plate
STLS  := $(addprefix $(OUT)/,$(addsuffix .stl,$(PARTS))) $(OUT)/assembly.stl

all: $(STLS)

$(OUT):
	mkdir -p $(OUT)

$(OUT)/finger_body.stl: finger_module.scad common.scad | $(OUT)
	$(OPENSCAD) $(FLAGS) -o $@ -D 'part="body"' $<
$(OUT)/finger_camera_cap.stl: finger_module.scad common.scad | $(OUT)
	$(OPENSCAD) $(FLAGS) -o $@ -D 'part="camera_cap"' $<
$(OUT)/finger_gel_frame.stl: finger_module.scad common.scad | $(OUT)
	$(OPENSCAD) $(FLAGS) -o $@ -D 'part="gel_frame"' $<
$(OUT)/rail_base.stl: gripper_housing.scad common.scad | $(OUT)
	$(OPENSCAD) $(FLAGS) -o $@ -D 'part="rail_base"' $<
$(OUT)/carriage_adapter.stl: gripper_housing.scad common.scad | $(OUT)
	$(OPENSCAD) $(FLAGS) -o $@ -D 'part="carriage_adapter"' $<
$(OUT)/trigger.stl: gripper_housing.scad common.scad | $(OUT)
	$(OPENSCAD) $(FLAGS) -o $@ -D 'part="trigger"' $<
$(OUT)/ft_plate.stl: gripper_housing.scad common.scad | $(OUT)
	$(OPENSCAD) $(FLAGS) -o $@ -D 'part="ft_plate"' $<
$(OUT)/assembly.stl: assembly.scad gripper_housing.scad finger_module.scad common.scad | $(OUT)
	$(OPENSCAD) $(FLAGS) -o $@ $<

check: all
	python3 check_stl.py $(OUT)

clean:
	rm -rf $(OUT)

.PHONY: all check clean
