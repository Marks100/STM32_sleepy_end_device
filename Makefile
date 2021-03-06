####################################################################################################
#                                  Ceedling test target & rules                                    #
####################################################################################################
GCOVR_EXE := gcovr
ARM		  := arm-none-eabi
CC		  := $(ARM)-gcc
LD		  := $(ARM)-gcc
OBJCOPY	  := $(ARM)-objcopy
AS        := $(ARM)-as
AR        := $(ARM)-ar
NM        := $(ARM)-nm
STRIP     := $(ARM)-strip


PROJECT_NAME	:= STM32_sleepy_sensors

GCC_ARM_OUT_DIR := Build_output
BUILD_SUPPORT   := Workspace/build
STM32_ELF_FILE  := $(PROJECT_NAME).elf
STM32_MAP_FILE  := $(PROJECT_NAME).map
STM32_COMPILER_OUTPUT := compile_log.txt
STM32_MEM_OUTPUT_FILE := memory_analysis.txt
RELEASE_PACKAGE_NAME :=  Release_package
STM32_MEM_OUTPUT_FILE := memory_analysis.txt

#MAJOR_SW = `awk '/MAJOR_SW/ { print $$3 }' Src/VERSIONS/VERSIONS.h`
#MINOR_SW = `awk '/MINOR_SW/ { print $$3 }' Src/VERSIONS/VERSIONS.h`

MAJOR_SW = `awk '/VERSION_MAJOR/ { print $$4+0 }' Src/autoversion.h`
MINOR_SW = `awk '/VERSION_PATCH/ { print $$4+0 }' Src/autoversion.h`
VERIFICATION_SW = `awk '/VERSION_VERIFICATION/ { print $$4+0 }' Src/autoversion.h`

# Various test reports
CEEDLING_TEST_XML_TEST_REPORT_ORIGIN   := test/build/artifacts/test/
CEEDLING_TEST_XML_TEST_REPORT_DEST 	   := CodeCoverage/Test_Report/
CEEDLING_GCOV_XML_TEST_REPORT_ORIGIN   := test/build/artifacts/gcov/
CEEDLING_GCOV_XML_TEST_REPORT_DEST     := CodeCoverage/GCOV/
CEEDLING_LCOV_XML_TEST_REPORT_DEST	   := CodeCoverage/LCOV/

# Output files
GCOV_OUTPUT_DIR := test/build/gcov/out

# Files to be excluded from gcov coverage
UNWANTED_GEN_COVERAGE :=        \
$(GCOV_OUTPUT_DIR)/cmock*       \
$(GCOV_OUTPUT_DIR)/SELMATH*     \
$(GCOV_OUTPUT_DIR)/CHKSUM*      \
$(GCOV_OUTPUT_DIR)/STDC*        \
$(GCOV_OUTPUT_DIR)/test_*       \
$(GCOV_OUTPUT_DIR)/unity.*      \
$(GCOV_OUTPUT_DIR)/*helper*     \

# location of autoversion header
AUTOVERS_HEADER := Src/autoversion.h

#Find all the source file in the given directories
SRCS := $(shell find $(STM32_SRC_DIRS) -type f -name '*.c')

#Transform the list of c files into a list of object files
OBJS := $(SRCS:.c=.o)

#add the "-I" to all folders found by the find command :)
INCLUDES := $(addprefix -I ,$(shell find $(STM32_SRC_DIRS) -type d))


CFLAGS :=  \
		-mcpu=cortex-m3 \
		-mthumb \
		-Wall \
		-ffunction-sections \
		-g \
		-O0 \
		-c \
		-DSTM32DRIVERS \
		-DSTM32F103C8 \
		-DSTM32F10X_MD \
		-DUSE_STDPERIPH_DRIVER \
		-D__ASSEMBLY__

LDFLAGS := \
		-mcpu=cortex-m3 \
		-mthumb \
		-g \
		-nostartfiles \
		"-Wl,-Map=$(GCC_ARM_OUT_DIR)/$(PROJECT_NAME).map" \
		-O0 \
		-Wl,--gc-sections \
		-L$(BUILD_SUPPORT)/ \
		-Wl,-T$(BUILD_SUPPORT)/arm-gcc-link.ld -g -o $(GCC_ARM_OUT_DIR)/$(PROJECT_NAME).elf \



.PHONY: all
all: GCC_ARM


.PHONY: GCC_ARM
GCC_ARM: build_clean $(AUTOVERS_HEADER) $(OBJS)
	@echo Compiling Completed...
	@mkdir -p $(GCC_ARM_OUT_DIR) $(GCC_ARM_OUT_DIR)/object_files/
	@echo "Linking Object Files..."
	@$(LD) $(LDFLAGS) $(OBJS)
	@echo "Linking Completed"
	@echo "$(PROJECT_NAME).map and $(PROJECT_NAME).elf files generated at: $(GCC_ARM_OUT_DIR)"
	@$(OBJCOPY) -O binary $(GCC_ARM_OUT_DIR)/$(PROJECT_NAME).elf $(GCC_ARM_OUT_DIR)/$(PROJECT_NAME).bin
	@$(OBJCOPY) -O ihex $(GCC_ARM_OUT_DIR)/$(PROJECT_NAME).elf $(GCC_ARM_OUT_DIR)/$(PROJECT_NAME).hex
	@echo "Copying object files to $(GCC_ARM_OUT_DIR) ...."
	@find . -type f -name "*.o" -print0 | xargs -0 -I{} cp "{}" -fr $(GCC_ARM_OUT_DIR)/object_files/
	@mv $(STM32_COMPILER_OUTPUT) $(GCC_ARM_OUT_DIR)
	@-sed -i '1 i\Compiler warnings generated by GCC for $(PROJECT_NAME) ' $(GCC_ARM_OUT_DIR)/$(STM32_COMPILER_OUTPUT)
	@-sed -i '2 i\ ' $(GCC_ARM_OUT_DIR)/$(STM32_COMPILER_OUTPUT)
	@echo "Build Succesfully Completed..."


# Creates Ceedling environment if it does not exist
test/vendor:
	@createCeedlingEnv

# Wild card test will allow any test to run once the name is provided
%.test: test/vendor
	echo $^
	@$(eval TEST_FILE := $(subst .test,.c,$@))
	@echo Testing $(TEST_FILE)...
	#cd test && rake test:$(TEST_FILE)


%.o: %.c
	@echo "Compiling $<"
	@$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@  2>&1 | tee -a $(STM32_COMPILER_OUTPUT)



.PHONY: test_all
test_all: test/vendor
	cd test && rake test:all
	@mkdir -p CodeCoverage/{Test_Report,LCOV,GCOV}
	@mv $(CEEDLING_TEST_XML_TEST_REPORT_ORIGIN)report.xml $(CEEDLING_TEST_XML_TEST_REPORT_DEST)
	@ceedling-gen-report $(CEEDLING_TEST_XML_TEST_REPORT_DEST)report.xml $(CEEDLING_TEST_XML_TEST_REPORT_DEST)SoftwareCeedlingTestReport.html
	@$(call check_test_result)


.PHONY: test_all_with_coverage
test_all_with_coverage: test/vendor
	@cd test/unit_test && rake gcov:all
	@mkdir -p CodeCoverage/{Test_Report,LCOV,GCOV}
	@mv -f $(CEEDLING_GCOV_XML_TEST_REPORT_ORIGIN)report.xml $(CEEDLING_TEST_XML_TEST_REPORT_DEST)
	@ceedling-gen-report $(CEEDLING_TEST_XML_TEST_REPORT_DEST)report.xml $(CEEDLING_TEST_XML_TEST_REPORT_DEST)SoftwareCeedlingTestReport.html
	@-rm -f $(UNWANTED_GEN_COVERAGE)


# Make Target: runs lcov coverage tool
.PHONY: gen_lcov_report
# Note: Test report checked for pass at end to ensure that test report is generated regardless of test case failures
gen_lcov_report: test_all_with_coverage
	@echo Generating LCOV reports.. Please be patient...
	@cd test && lcov -t 'LCov_report' -o ../CodeCoverage/LCOV/output_file.info -c -d . --rc lcov_branch_coverage=1
	@cd $(CEEDLING_LCOV_XML_TEST_REPORT_DEST) && genhtml -o . output_file.info --rc lcov_branch_coverage=1 --sort --show-details
ifdef ConEmuDir
	@cd $(CEEDLING_LCOV_XML_TEST_REPORT_DEST) && start index.html
endif
	@$(call check_test_result)




# user fn check overall test result for Pass \ Fail.  Exits with error code on failure
define check_test_result
	@if grep -q "Overall Test Result: PASS" $(CEEDLING_TEST_XML_TEST_REPORT_DEST)SoftwareCeedlingTestReport.html; \
	then \
	  echo Overall Test Result: PASS; \
	else \
	  echo Overall Test Result: FAIL; \
	  exit 1; \
	fi
endef



####################################################################################################
#                                  Miscellaneous targets & rules                                   #
####################################################################################################

# Print all targets if no target has been supplied
.PHONY: no_targets__ list
no_targets__:
list:
	@echo "Available Targets:"
	@sh -c "$(MAKE) -p no_targets__ | awk -F':' '/^[a-zA-Z0-9][^\$$#\/\\t=]*:([^=]|$$)/ {split(\$$1,A,/ /);for(i in A)print A[i]}' | grep -v '__\$$' | sort"


$(AUTOVERS_HEADER):
	sVersion --autoversion

$(GCC_ARM_OUT_DIR)/$(STM32_MAP_FILE):
	@echo "$(STM32_MAP_FILE) does not exist, generating now"
	@$(MAKE) -s GCC_ARM


.PHONY: total_clean
total_clean: build_clean
	@cd test && rm -fR build vendor
	@-rm -fR CodeCoverage
	@echo Clean Complete..



.PHONY: build_clean
build_clean:
	@echo cleaning up old build objects..... Please wait.
	@find ../ -type f -name '*THE_RAIN*' -print0 | xargs -0 rm -f
	@find . -type f -name '*.o*' -print0 | xargs -0 rm -f
	@- rm -fr $(GCC_ARM_OUT_DIR)
	@-rm -f $(AUTOVERS_HEADER)
	@echo Build cleaned..




# Set memory limits to match the Resource Usage in the Software Integration Spec

# ROM limits
ROM_SIZE := 65536
ROM_PERC_LIMIT := 80
ROM_CALC_LIMIT := $(shell echo $$(($(ROM_SIZE) * $(ROM_PERC_LIMIT) / 100 )))

# RAM limits
RAM_SIZE := 20480
RAM_PERC_LIMIT := 80
RAM_CALC_LIMIT := $(shell echo $$(($(RAM_SIZE) * $(RAM_PERC_LIMIT) / 100 )))


.PHONY: memory_stats
memory_stats: $(GCC_ARM_OUT_DIR)/$(STM32_MAP_FILE)
	@echo "Analysing elf file for memory stats...."
	@echo "Output from GNU \"size\" tool.." | tee $(GCC_ARM_OUT_DIR)/$(STM32_MEM_OUTPUT_FILE)
	@size -B $(GCC_ARM_OUT_DIR)/$(STM32_ELF_FILE) | tee -a $(GCC_ARM_OUT_DIR)/$(STM32_MEM_OUTPUT_FILE)

	@awk 'BEGIN{ printf "\n=======================================================\n\
	====================== RAM STATS ======================\n\
	=======================================================\n\n"; } \
	/$(STM32_ELF_FILE)/ { RamUsed = ($$2) + ($$3); } \
	END{ printf "Available Ram in Bytes: %-6d\nRam used in Bytes:      %-6d\n%%Ram used:              %-3.0f%\n", $(RAM_SIZE), RamUsed, ( RamUsed/$(RAM_SIZE) * 100 ); }' $(GCC_ARM_OUT_DIR)/$(STM32_MEM_OUTPUT_FILE) | tee -a $(GCC_ARM_OUT_DIR)/$(STM32_MEM_OUTPUT_FILE)

	@awk 'BEGIN{ printf "\n=======================================================\n\
	==================== FLASH STATS ======================\n\
	=======================================================\n\n"; } \
	/$(STM32_ELF_FILE)/ { FlashUsed = ($$1) ;} \
	END{ printf "Available Flash in Bytes: %-6d\nFlash used in Bytes:      %-6d\n%%Flash used:              %-3.0f%\n\
	-------------------------------------------------------\n\n", $(ROM_SIZE), FlashUsed, ( FlashUsed/$(ROM_SIZE) * 100 ); }' $(GCC_ARM_OUT_DIR)/$(STM32_MEM_OUTPUT_FILE) | tee -a $(GCC_ARM_OUT_DIR)/$(STM32_MEM_OUTPUT_FILE)
	@echo "Output file \"$(STM32_MEM_OUTPUT_FILE)\" created @ $(PROJECT_NAME)/$(GCC_ARM_OUT_DIR)/"



chksum:
	@find Src -type f -print0 | xargs -0 sha1sum > output.txt



.PHONY: package
package:
	@echo Creating release package
	@-rm -fr $(RELEASE_PACKAGE_NAME)
	@mkdir -p $(RELEASE_PACKAGE_NAME)
	@$(MAKE) -s GCC_ARM
	@echo Generating memory stats...
	@$(MAKE) -s memory_stats > /dev/null
	@echo Copying build output to $(RELEASE_PACKAGE_NAME) folder..
	@cp -r $(GCC_ARM_OUT_DIR)/. $(RELEASE_PACKAGE_NAME)
	@echo "copying version file info"
	@cp -f $(AUTOVERS_HEADER) $(RELEASE_PACKAGE_NAME)
	@touch $(RELEASE_PACKAGE_NAME)/SW_V$(MAJOR_SW).$(MINOR_SW).$(VERIFICATION_SW)_BETA
	@find Src -type f -print0 | xargs -0 sha1sum > $(RELEASE_PACKAGE_NAME)/chksum.txt
	@echo Checksum file generated..
	@echo zipping up the release package..
	@-7za a "$(RELEASE_PACKAGE_NAME)/$(RELEASE_PACKAGE_NAME)_V$(MAJOR_SW).$(MINOR_SW).$(VERIFICATION_SW)_BETA.zip" $(RELEASE_PACKAGE_NAME)/* > /dev/null
	@find $(RELEASE_PACKAGE_NAME) ! -name '$(RELEASE_PACKAGE_NAME)*' -print0 | xargs -0 rm -fr
	@echo "Release package created V$(MAJOR_SW).$(MINOR_SW).$(VERIFICATION_SW)_BETA"
