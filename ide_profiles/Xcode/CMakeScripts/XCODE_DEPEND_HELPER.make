# DO NOT EDIT
# This makefile makes sure all linkable targets are
# up-to-date with anything they link to
default:
	echo "Do not invoke directly"

# Rules to remove targets that are older than anything to which they
# link.  This forces Xcode to relink the targets from scratch.  It
# does not seem to check these dependencies itself.
PostBuild.mpc.Debug:
/Users/srikanthnarayanan/Documents/GitHubRepos/CarND-MPC/ide_profiles/Xcode/Debug/mpc:
	/bin/rm -f /Users/srikanthnarayanan/Documents/GitHubRepos/CarND-MPC/ide_profiles/Xcode/Debug/mpc


PostBuild.mpc.Release:
/Users/srikanthnarayanan/Documents/GitHubRepos/CarND-MPC/ide_profiles/Xcode/Release/mpc:
	/bin/rm -f /Users/srikanthnarayanan/Documents/GitHubRepos/CarND-MPC/ide_profiles/Xcode/Release/mpc


PostBuild.mpc.MinSizeRel:
/Users/srikanthnarayanan/Documents/GitHubRepos/CarND-MPC/ide_profiles/Xcode/MinSizeRel/mpc:
	/bin/rm -f /Users/srikanthnarayanan/Documents/GitHubRepos/CarND-MPC/ide_profiles/Xcode/MinSizeRel/mpc


PostBuild.mpc.RelWithDebInfo:
/Users/srikanthnarayanan/Documents/GitHubRepos/CarND-MPC/ide_profiles/Xcode/RelWithDebInfo/mpc:
	/bin/rm -f /Users/srikanthnarayanan/Documents/GitHubRepos/CarND-MPC/ide_profiles/Xcode/RelWithDebInfo/mpc




# For each target create a dummy ruleso the target does not have to exist
